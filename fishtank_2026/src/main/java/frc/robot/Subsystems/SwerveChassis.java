package frc.robot.Subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.k_chassis;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Rotation2d;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.wpilibj.XboxController;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

import frc.robot.commands.SwerveDriveCommand;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

public class SwerveChassis extends SubsystemBase {



    private SwerveWheel leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel;

    // Gyro for robot orientation
    private ChassisGyro gyro;
    private Field2d m_odoField;
    private Pose2d m_pose;
    private boolean m_fieldForward = true;

    
    private double worldRotation;
    public static double[] chassisLength = {0.505, Units.inchesToMeters(22.75)}; // index 0 is 2026, 1 is 2025
    public static double[] chassisWidth = {0.630, Units.inchesToMeters(22.75)};
    // public static double[] chassisLength = {0.630, Units.inchesToMeters(22.75)}; // index 0 is 2026, 1 is 2025
    // public static double[] chassisWidth = {0.505, Units.inchesToMeters(22.75)};
   
    public SwerveDriveKinematics m_driveKinematics;
    public SwerveDriveOdometry m_odometry;

    public void setWorldRotation(double nWR)
    {
        //worldRotation = nWR;
        //worldRotation = nWR;
        worldRotation = 0;
        //SmartDashboard.putNumber("WorldRotation", worldRotation);
        
        

        periodic();
        Rotation2d gyroAngle = new Rotation2d(Math.toRadians(gyro.getAngle()));
         m_odometry.resetPosition(gyroAngle,
        new SwerveModulePosition[] {
        leftFrontWheel.getPosition(), rightFrontWheel.getPosition(),
        leftRearWheel.getPosition(), rightRearWheel.getPosition()
        }, m_pose);
    
    } 
    public double getWorldRotation()
    {
        return worldRotation;
    }
    
    public void waitResetGyro() {
        new Thread(() -> {
            try {
                Thread.sleep(1000);
                gyro.reset();
            } catch (Exception e) {
            }
        }).start();
    }

    public void fieldForward(boolean isForward)
    {
        m_fieldForward = isForward;
    }

    public Rotation2d getRotation2d() {
        double angle = 0;
        if (m_fieldForward)
        {
            angle = gyro.getAngle();
        }
        SmartDashboard.putNumber("bot gyro", angle);
        SmartDashboard.putBoolean("fieldForward", m_fieldForward);

        return Rotation2d.fromDegrees(angle); //angle - 90
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
    public SwerveChassis(XboxController controller, ChassisGyro gyro, Field2d field) {
       
        this.gyro = gyro;
        waitResetGyro();

        SparkFlexConfig configLeft = new SparkFlexConfig();
        SparkFlexConfig configRight = new SparkFlexConfig();
       
        m_odoField = field;
        //m_odoField.setRobotPose(2,3,Rotation2d.fromDegrees(0.0));


        configLeft.idleMode(IdleMode.kBrake);
        configLeft.encoder.positionConversionFactor(1.0);
        configLeft.encoder.velocityConversionFactor(1.0);
        configLeft.smartCurrentLimit(Constants.k_chassis.kCurrentLimit);

        configLeft.inverted(false);
        configRight.apply(configLeft);

        configRight.inverted(false); //usually false
        if (Constants.hasCanCoder)
        {
            m_driveKinematics = new SwerveDriveKinematics(
                new Translation2d(chassisLength[Constants.enc] / 2, chassisWidth[Constants.enc] / 2), //kWheelBase then kTrackWidth
                new Translation2d(chassisLength[Constants.enc] / 2, -chassisWidth[Constants.enc] / 2), 
                new Translation2d(-chassisLength[Constants.enc] / 2, chassisWidth[Constants.enc] / 2), 
                new Translation2d(-chassisLength[Constants.enc] / 2, -chassisWidth[Constants.enc] / 2)); 
        }
        else
        {
            m_driveKinematics = new SwerveDriveKinematics(
                new Translation2d(chassisLength[Constants.enc] / 2, -chassisWidth[Constants.enc] / 2), //kWheelBase then kTrackWidth
                new Translation2d(chassisLength[Constants.enc] / 2, chassisWidth[Constants.enc] / 2), 
                new Translation2d(-chassisLength[Constants.enc] / 2, -chassisWidth[Constants.enc] / 2), 
                new Translation2d(-chassisLength[Constants.enc] / 2, chassisWidth[Constants.enc] / 2)); 
        }


        leftFrontWheel = new SwerveWheel("LF", Constants.k_chassis.leftFrontMotorPort,
                                         Constants.k_chassis.leftFrontMotorSteerPort,
                                         Constants.leftFrontAbsEncPort[Constants.enc] /* move to Constants */, configLeft,
                                         Constants.leftFrontAbsOffset);
        rightFrontWheel = new SwerveWheel("RF", Constants.k_chassis.rightFrontMotorPort,
                                         Constants.k_chassis.rightFrontMotorSteerPort,
                                         Constants.rightFrontAbsEncPort[Constants.enc] /* move to Constants */, configRight,
                                         Constants.rightFrontAbsOffset);
        leftRearWheel = new SwerveWheel("LR", Constants.k_chassis.leftRearMotorPort,
                                         Constants.k_chassis.leftRearMotorSteerPort,
                                         Constants.leftRearAbsEncPort[Constants.enc] /* move to Constants */, configLeft,
                                         Constants.leftRearAbsOffset);
        rightRearWheel = new SwerveWheel("RR", Constants.k_chassis.rightRearMotorPort,
                                         Constants.k_chassis.rightRearMotorSteerPort,
                                         Constants.rightRearAbsEncPort[Constants.enc] /* move to Constants */, configRight,
                                         Constants.rightRearAbsOffset);


        //motors for odometry
        m_odometry = new SwerveDriveOdometry(
        m_driveKinematics, gyro.getRotation2d(),
        new SwerveModulePosition[] {
            leftFrontWheel.getPosition(),
            rightFrontWheel.getPosition(),
            leftRearWheel.getPosition(),
            rightRearWheel.getPosition()
        }, new Pose2d(5.0, 6.5, new Rotation2d())); //x and y is robot starting position in field

        setDefaultCommand(new SwerveDriveCommand(this, controller, gyro));
        
    }
      
    public void testMotors(double roll_speed, double rot_speed) {
     

        SmartDashboard.putNumber("Roll", roll_speed);
        SmartDashboard.putNumber("Rotation", rot_speed);

        leftFrontWheel.setMotors(roll_speed, rot_speed);
        rightFrontWheel.setMotors(roll_speed, rot_speed);
        leftRearWheel.setMotors(roll_speed, rot_speed);
        rightRearWheel.setMotors(roll_speed, rot_speed);

        

        turnWheel(angleTest);
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

    public SwerveDriveKinematics driveKinematics()
    {
        return m_driveKinematics;
    }



    @Override
    public void periodic() {
        Rotation2d gyroAngle = new Rotation2d(Math.toRadians(gyro.getAngle()));
       m_pose = m_odometry.update(gyroAngle,
        new SwerveModulePosition[] {
      leftFrontWheel.getPosition(), rightFrontWheel.getPosition(),
      leftRearWheel.getPosition(), rightRearWheel.getPosition()
    });
    
   m_odoField.setRobotPose(m_pose);
    SmartDashboard.putData("OdoField", m_odoField);
    SmartDashboard.putString("odometry gyro", gyroAngle.toString());
    SmartDashboard.putNumber("Robot X", Math.round(m_pose.getX() * 100.0)/100.0);
    SmartDashboard.putNumber("Robot Y", Math.round(m_pose.getY() * 100.0) / 100.0);
  
    SmartDashboard.putString("rotation2d", getRotation2d().toString());
      
        // Update the dashboard with the gyro angle for debugging
        SmartDashboard.putNumber("Gyro Angle", gyro.getAngle());


        rightFrontWheel.publishData();
        rightRearWheel.publishData();
        leftFrontWheel.publishData();
        leftRearWheel.publishData();

     
    }

}
