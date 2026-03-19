package frc.robot.Subsystems;

import org.ejml.dense.block.MatrixOps_DDRB;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.Constants;
import frc.robot.Constants.k_chassis;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;


public class SwerveWheel {
    // Each wheel has a motor for driving and another for steering
    private SparkFlex driveMotor, steerMotor;

    // Each motor has a built-in encoder so we can tell how far it has turned
    private RelativeEncoder driveEncoder;//, steerEncoder;

    // Each wheel also has a "through-bore" encoder to give an absolute reference point
    private SwerveEncoder absoluteEncoder;
    private int absEncPort;

    // Use PID to control steering.  This will make the wheel turn faster when it has
    // a longer way to turn, and slower when it gets close to the target angle
    private PIDController steerPID;
    private PIDController drivePID;

    private boolean targetDrivePositionSet;
    private double targetDrivePosition;
    private double targetDriveRadians;
    private double targetDriveDirection;

    private SwerveModulePosition m_currentPosition;
    // A label for the dashboard that describes this wheel's location
    private String loc;

    private boolean drivePidActive;

   



    // Constructor - needs ports for all motors and the through-bore encoder for
    // this wheel.  Also needs a motor config
    public SwerveWheel(String location, int drivePort, int steerPort, int absPort,
                       SparkFlexConfig config, double absoluteOffset) {
      
        loc = location;
        
        steerMotor = new SparkFlex(steerPort, MotorType.kBrushless);

      
        steerMotor.configure(config, ResetMode.kResetSafeParameters,PersistMode.kPersistParameters);
        
        // Set up both motors
        driveMotor  = new SparkFlex(drivePort, MotorType.kBrushless);
        driveMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);  // Needed?
        
        // Set up all three encoders
        driveEncoder = driveMotor.getEncoder();
        //steerEncoder = steerMotor.getEncoder();

        targetDrivePositionSet = false;
        targetDrivePosition = 0.0;
        targetDriveRadians = 0.0;
        targetDriveDirection = 1.0;
        drivePidActive=false;
    

        absEncPort = absPort;
        absoluteEncoder = new SwerveEncoder(absPort, 1.0, 0.0, absoluteOffset);
    

        // Create the PID controller for steering the wheel
        steerPID = new PIDController(k_chassis.kPTurning, 0, 0);
        steerPID.enableContinuousInput(-Math.PI, Math.PI);

        // Create PID controller for driving the wheel
        drivePID = new PIDController(k_chassis.kPDrive, k_chassis.kI, k_chassis.kD);

        
        
        resetEncoders();

       // m_currentPosition = new SwerveModulePosition(0,new Rotation2d(0));
    }


    // Return the absolute encoder value in radians.
    public double getAbsEncoderRad() {
        return absoluteEncoder.get();
    }

    public double getDrivePosition() {
        return driveEncoder.getPosition()/Constants.driveEncoderRatio;
    }

    public double getDriveTargetPosition() {
        if (targetDrivePositionSet) {
            return targetDrivePosition;
        }
        return getDrivePosition();
    }

    public void setDriveTargetRelativePosition(double relPos, double radians) {
        targetDrivePosition = getDrivePosition() + relPos;
        targetDriveRadians = radians;
        if (relPos > 0.0) {
            targetDriveDirection = 1.0;
        }
        else {
            targetDriveDirection = -1.0;
        }
        drivePID.reset();
        drivePidActive = false;
        targetDrivePositionSet = true;
    }

    public double getTurningPosition() {
        return absoluteEncoder.get(); 
    }
    

    public double getDriveVelocity() {
        return driveEncoder.getVelocity();
    }

    // public SwerveModulePosition getPosition()
    // {
    //     return m_currentPosition;
    // }
    public SwerveModulePosition getPosition(){
       // SmartDashboard.putNumber(loc + "driveEnc * 1", driveEncoder.getPosition() * Constants.k_chassis.kDriveEncoderDistancePerRotation);
        SmartDashboard.putNumber("drive position", -driveEncoder.getPosition());
        return new SwerveModulePosition(
            -driveEncoder.getPosition() * Constants.k_chassis.kDriveEncoderDistancePerRotation,
           // Rotation2d.fromRadians(0.0)
            Rotation2d.fromRadians(getAbsEncoderRad())
        );
        
    }
//     public SwerveModulePosition getPosition() {
//      return new SwerveModulePosition(
//         driveEncoder.getDistance(), new Rotation2d(steerEncoder.getDistance()));
//   }

    // public double getTurningVelocity(RelativeEncoder turningEncoder) {
    //     return steerEncoder.getVelocity();
    // }

    public void resetEncoders() {
            new Thread(() -> {
                try {
                    Thread.sleep(2000);
                    double tmp = getAbsEncoderRad();
                    SmartDashboard.putNumber(loc+"InitialAdj", tmp);
                    driveEncoder.setPosition(0);
                   // steerEncoder.setPosition(tmp* Constants.steerEncoderRatio); 
                } catch (Exception e) {
                }
            }).start();
        }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void turnWheel(double angle) {
        double turnSpeed = steerPID.calculate(getTurningPosition(), angle);
        if (Math.abs(turnSpeed) > 0.0005) {
            steerMotor.set(turnSpeed);
        } else {
            stop();
        }
    }

    public void publishData() {
        SmartDashboard.putNumber(loc+" Port", absEncPort);
        if (Constants.hasPWMEncoder)
        {
            SmartDashboard.putNumber(loc + " Raw Angle", this.absoluteEncoder.getRawDutyCycle());
        }
        
        SmartDashboard.putNumber(loc+" Steer Encoder Val", Math.round(getAbsEncoderRad() * 100.0) /100.0);

        
    }

    public void setState(SwerveModuleState state, boolean manualControl) {
        // Need to convert the velocity and angle settings to the proper motor settings using
        // rate limiters and PID controls
        // Set drive motor speeds (using percentage output or velocity mode)
        if (manualControl){
            if (Math.abs(state.speedMetersPerSecond) < 0.001){
                stop();
                return;
            }
            state = SwerveModuleState.optimize(state, getState().angle);
            double metersPerSecond = state.speedMetersPerSecond;
            double angle = state.angle.getRadians();

            driveMotor.set(state.speedMetersPerSecond);
            // Set steering motor positions (we use position control to set the angle)
            steerMotor.set(steerPID.calculate(getTurningPosition(), state.angle.getRadians()));
        }
        else {
            double curPos = getDrivePosition();
            double targPos = getDriveTargetPosition();
            double delta = Math.abs(curPos-targPos);

            SmartDashboard.putNumber(loc+"Current Pos", curPos);
            SmartDashboard.putNumber(loc+"Target Pos", targPos);
            SmartDashboard.putNumber(loc+"Delta", curPos);

            turnWheel(targetDriveRadians);

            if (delta < k_chassis.targetReached) {
              targetDrivePositionSet = false;
              stop();
            }
            else if (drivePidActive) {
                double setSpeed = drivePID.calculate(curPos, targPos);
                setSpeed *= k_chassis.activatePidSpeedScaler;
                driveMotor.set(setSpeed);
              }
            else {
              drivePidActive = delta < k_chassis.activateDrivePid;
              driveMotor.set(k_chassis.MaxMetersPerSecond*targetDriveDirection);
            }
        }
    }

    public void setMotors(double roll_speed, double rot_speed) {
        // Limit speeds for testing to try to limit runaway motors
        if (roll_speed < -0.2) {roll_speed = -0.2;}
        if (roll_speed > 0.2) {roll_speed = 0.2;}
        if (rot_speed < -0.2) {rot_speed = -0.2;}
        if (rot_speed > 0.2) {rot_speed = 0.2;}

        driveMotor.set(roll_speed);
        steerMotor.set(rot_speed);
    }
    
    public void stop() {
        driveMotor.set(0);
        steerMotor.set(0);
    }

   

}
