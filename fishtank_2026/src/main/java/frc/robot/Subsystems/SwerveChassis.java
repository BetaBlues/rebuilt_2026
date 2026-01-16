package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.k_chassis;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.commands.SwerveDriveCommand;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;


public class SwerveChassis extends SubsystemBase {

    // Constants for track width and wheel base (in meters)
    private final double kTrackWidth = 0.5; // meters
    private final double kWheelBase = 0.5; // meters

    private SwerveWheel leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel;

    // Gyro for robot orientation
    private ChassisGyro gyro;
    
    // Controller for driving robot
    private XboxController controller;

    // Swerve Drive Kinematics (for calculating wheel speeds and angles)
    private SwerveDriveKinematics m_kinematics;


    
    public void resetGyro() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();
            } catch (Exception e) {
            }
        }).start();
    }

    public Rotation2d getRotation2d() {
        double angle = Math.IEEEremainder(gyro.getAngle(), 360);
        SmartDashboard.putNumber("bot gyro", angle);

        return Rotation2d.fromDegrees(angle - 90);
    }
    private int angleTest = 0;
    public void turnWheel(int degrees) {
       double angle = Math.toRadians(degrees);
       rightFrontWheel.turnWheel(angle);
       leftFrontWheel.turnWheel(angle);
       leftRearWheel.turnWheel(angle);
       rightRearWheel.turnWheel(angle);
       angleTest = degrees;
    }
    public SwerveChassis(XboxController controller, ChassisGyro gyro) {
       

        this.controller = controller;
        this.gyro = gyro;
        resetGyro();

        SparkFlexConfig configLeft = new SparkFlexConfig();
        SparkFlexConfig configRight = new SparkFlexConfig();

        configLeft.idleMode(IdleMode.kCoast);
        // configLeft.idleMode(IdleMode.kCoast);
        configLeft.encoder.positionConversionFactor(1.0);
        configLeft.encoder.velocityConversionFactor(1.0);
        configLeft.smartCurrentLimit(Constants.k_chassis.kCurrentLimit);

        configLeft.inverted(false);
        configRight.apply(configLeft);

        configRight.inverted(false);

        leftFrontWheel = new SwerveWheel("LF", Constants.k_chassis.leftFrontMotorPort,
                                         Constants.k_chassis.leftFrontMotorSteerPort,
                                         Constants.leftFrontAbsEncPort /* move to Constants */, configLeft,
                                         Constants.leftFrontAbsOffset);
        rightFrontWheel = new SwerveWheel("RF", Constants.k_chassis.rightFrontMotorPort,
                                         Constants.k_chassis.rightFrontMotorSteerPort,
                                         Constants.rightFrontAbsEncPort /* move to Constants */, configRight,
                                         Constants.rightFrontAbsOffset);
        leftRearWheel = new SwerveWheel("LR", Constants.k_chassis.leftRearMotorPort,
                                         Constants.k_chassis.leftRearMotorSteerPort,
                                         Constants.leftRearAbsEncPort /* move to Constants */, configLeft,
                                         Constants.leftRearAbsOffset);
        rightRearWheel = new SwerveWheel("RR", Constants.k_chassis.rightRearMotorPort,
                                         Constants.k_chassis.rightRearMotorSteerPort,
                                         Constants.rightRearAbsEncPort /* move to Constants */, configRight,
                                         Constants.rightRearAbsOffset);

        setDefaultCommand(new SwerveDriveCommand(this, controller));
    }
      
    public void testMotors(double roll_speed, double rot_speed) {
        double gyroAngle = gyro.getAngle();
        double radianAngle = Math.toRadians(gyroAngle);
     

        SmartDashboard.putNumber("Roll", roll_speed);
        SmartDashboard.putNumber("Rotation", rot_speed);

        leftFrontWheel.setMotors(roll_speed, rot_speed);
        rightFrontWheel.setMotors(roll_speed, rot_speed);
        leftRearWheel.setMotors(roll_speed, rot_speed);
        rightRearWheel.setMotors(roll_speed, rot_speed);

        turnWheel(angleTest);
    }
    public void zeroHeading() {
        gyro.reset();
    }
    public void setModuleStates(SwerveModuleState[] states, boolean manualState) {
        // Note: need to normalize drive speeds here using kinematics.normalizeWheelSpeeds() !
        SwerveDriveKinematics.desaturateWheelSpeeds(states, k_chassis.MaxMetersPerSecond);
        // Then pass calculated drive states down to individual wheels
        leftFrontWheel.setState(states[0], manualState);
        rightFrontWheel.setState(states[1], manualState);
        leftRearWheel.setState(states[2], manualState);
        rightRearWheel.setState(states[3], manualState);
    }

    public void setDriveTargetRelativePosition(double relPos, double degrees){
        leftFrontWheel.setDriveTargetRelativePosition(relPos, Math.toRadians(degrees));
        rightFrontWheel.setDriveTargetRelativePosition(relPos, Math.toRadians(degrees));
        leftRearWheel.setDriveTargetRelativePosition(relPos, Math.toRadians(degrees));
        rightRearWheel.setDriveTargetRelativePosition(relPos, Math.toRadians(degrees));
    }

    public void advance(double posX, double posY) {
    }

    public void drive(double xSpeed, double ySpeed, double rot) {
        // Get the robot's current orientation (yaw) from the gyro
        double currentAngle = gyro.getAngle(); // In degrees

        // Convert joystick inputs to field-relative speeds
        // Transform the joystick values from robot-relative to field-relative
        // coordinates
        double tempX = xSpeed * Math.cos(Math.toRadians(currentAngle))
                + ySpeed * Math.sin(Math.toRadians(currentAngle));
        double tempY = -xSpeed * Math.sin(Math.toRadians(currentAngle))
                + ySpeed * Math.cos(Math.toRadians(currentAngle));
        
    }

    public void driveTime(double seconds, double kAutoDriveSpeed, double kAutoRotationSp) {
        new Thread(() -> {
            try {
                leftFrontWheel.setMotors(kAutoDriveSpeed, kAutoRotationSp);
                rightFrontWheel.setMotors(kAutoDriveSpeed, kAutoRotationSp);
                leftRearWheel.setMotors(kAutoDriveSpeed, kAutoRotationSp);
                rightRearWheel.setMotors(kAutoDriveSpeed, kAutoRotationSp);
                Thread.sleep((long) (seconds));
                stopModules();
            } catch (InterruptedException e) {
                e.printStackTrace();
                stopModules();
            }
        }).start();
    }


    public void stopModules() {
        rightFrontWheel.stop();
        rightRearWheel.stop();
        leftFrontWheel.stop();
        leftRearWheel.stop();
    }

    @Override
    public void periodic() {
        // Update the dashboard with the gyro angle for debugging
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());

        rightFrontWheel.publishData();
        rightRearWheel.publishData();
        leftFrontWheel.publishData();
        leftRearWheel.publishData();
    }

}
