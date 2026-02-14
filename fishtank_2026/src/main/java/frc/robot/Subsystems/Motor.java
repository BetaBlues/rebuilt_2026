package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants;
import frc.robot.Constants.SeeSawConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.units.Units;
import edu.wpi.first.units.measure.Distance;
//import edu.wpi.first.units.measure.;






public class Motor extends SubsystemBase{
    private SparkMax spinMotor;
    private RelativeEncoder motorEncoder;
    private SparkMaxConfig config;
    private double m_setpoint = 0.0; //was 70.0
    private boolean m_Active = false;
    private PIDController spinMotorPID;
    private SysIdRoutine routine;
    private SparkClosedLoopController motorClosedLoop;
    private String motorName;
    private double targetVelocity;
    private double targetPosition;

    private SysIdRoutine log;
    //private final SysIdRoutine m_sysIdRoutine;


    public void sysIdDrive(Voltage outputVolts)
    {
        spinMotor.setVoltage(outputVolts);
    }

    public void sysIdLog(SysIdRoutineLog stuff)
    {
        

    }

    public Motor(String name, int canId) {
        motorName = name;

    SysIdRoutine routine = new SysIdRoutine(
        new SysIdRoutine.Config(), 
        new SysIdRoutine.Mechanism(this::sysIdDrive, this::sysIdLog, this)
        );    

    spinMotor = new SparkMax(canId, MotorType.kBrushless);
    motorEncoder = spinMotor.getEncoder();
    config = new SparkMaxConfig();
    motorClosedLoop = spinMotor.getClosedLoopController();
    double batteryVoltage = RobotController.getBatteryVoltage();
    
    

    config.idleMode(IdleMode.kBrake);
    // config.encoder.positionConversionFactor(44.77);
    config.encoder.positionConversionFactor(1.0);
    config.encoder.velocityConversionFactor(1.0/1000);//1.0/5676);
    config.smartCurrentLimit(Constants.SpinMotorConstants.kCurrentLimit); // edit depending on arm vs seesaw

    config.softLimit.forwardSoftLimit(Constants.SpinMotorConstants.kSoftLimitForward);
    config.softLimit.reverseSoftLimit(Constants.SpinMotorConstants.kSoftLimitReverse);
    config.softLimit.forwardSoftLimitEnabled(true);
    config.softLimit.reverseSoftLimitEnabled(true);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    config.inverted(false);

    // Create the PID controller for turning the arm
    // spinMotorPID = new PIDController(Constants.SpinMotorConstants.kPSSRotation, 0, 0);
    // spinMotorPID.enableContinuousInput(-180, 180);
    
    config.closedLoop.pid(Constants.SpinMotorConstants.pos_kP, Constants.SpinMotorConstants.pos_kI, Constants.SpinMotorConstants.pos_kD, ClosedLoopSlot.kSlot0);
    config.closedLoop.pid(Constants.SpinMotorConstants.vel_kP, Constants.SpinMotorConstants.vel_kI, Constants.SpinMotorConstants.vel_kD, ClosedLoopSlot.kSlot1);
    spinMotor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); 
        try {Thread.sleep(3000);
        } catch (Exception e) {
        }
        }
        
   
    

    public void setPositionZero() {
        motorEncoder.setPosition(0);
    }

    public double getMotorPosition() {
        return motorEncoder.getPosition();
    }

    public double getMotorVelocity() {
        return motorEncoder.getVelocity();
    }

    public double getBusVoltage() {
        return spinMotor.getBusVoltage();
    }

    public void setTargetPosition(double pGoal, boolean relative) {
        if (relative)
        {
            double curPos = getMotorPosition();
            pGoal += curPos;
        }
        motorClosedLoop.setReference(pGoal, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        motorClosedLoop.setIAccum(0);
        targetPosition = pGoal;
        
        // if (curPos != pGoal) {
        //     m_setpoint = pGoal;
        //     m_Active = true; // this needs to be true to enable the motor
        System.out.println("proceeding to setpoint = " + pGoal);
        //}
    }

    public void setTargetVelocity(double pGoal, boolean relative) {
        if (relative)
        {
            double curVel = getMotorVelocity();
            pGoal += curVel;
        }
        motorClosedLoop.setReference(pGoal, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        motorClosedLoop.setIAccum(0);
        targetVelocity = pGoal;
        
        // if (curPos != pGoal) {
        //     m_setpoint = pGoal;
        //     m_Active = true; // this needs to be true to enable the motor
        System.out.println("proceeding to setpoint = " + pGoal);
        //}
    }

    public void MovePos(double angle, boolean relative) {
        setTargetPosition(angle, relative);
        
    }

    public void updateDashboard() {
        SmartDashboard.putNumber(motorName + " Current Velocity", this.getMotorVelocity());
        SmartDashboard.putNumber(motorName + " Target Velocity", targetVelocity);
        SmartDashboard.putNumber(motorName + " Current Position", this.getMotorPosition());
        SmartDashboard.putNumber(motorName + " Target Position", targetPosition);
        SmartDashboard.putNumber(motorName + " Bus Velocity", this.getBusVoltage());

    }



    public void MovePos(double angle) {
        setTargetPosition(angle, false);
    }


    



    // turns the arm
    public void MoveMotor(double speed) {
        spinMotor.set(speed);
    }

    
        public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return routine.dynamic(direction);
        }

        
        public Command sysIdQuasistatic(Direction direction) {
            return routine.quasistatic(direction);
        }

        public void stopMotor() {
            System.out.println("Stop Motor");
            // System.out.println("current position = " + seeSawEncoder.getPosition());
            spinMotor.stopMotor();
        }

}

        