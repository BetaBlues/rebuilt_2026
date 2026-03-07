package frc.robot.Subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkClosedLoopController;

import frc.robot.Constants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.sysid.SysIdRoutineLog;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import edu.wpi.first.wpilibj.RobotController;






public class Motor extends SubsystemBase{
    private SparkMax spinMotor;
    private RelativeEncoder motorEncoder;
    private SparkMaxConfig config;
    private double m_setpoint = 0.0; //was 70.0
    private boolean m_Active = false;
    private PIDController motorPID;
    private SysIdRoutine routine;
    private SparkClosedLoopController motorClosedLoop;
    private String motorName;
    private double targetVelocity;
    private double targetPosition;
    private PIDController climberPID;
    private double m_eSetpoint = 0.0;
    private String loc;

    private SysIdRoutine log;
  


    public void sysIdDrive(Voltage outputVolts)
    {
        spinMotor.setVoltage(outputVolts);
    }

    public void sysIdLog(SysIdRoutineLog stuff)
    {
        

    }

    public Motor(String name, int canId, double kpPID, double kiPID, double kdPID) {
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
   
    config.encoder.positionConversionFactor(1.0);
    config.encoder.velocityConversionFactor(1.0/1000);//1.0/5676);
    config.smartCurrentLimit(Constants.SpinMotorConstants.kCurrentLimit); // edit depending on arm vs seesaw

    config.softLimit.forwardSoftLimit(Constants.SpinMotorConstants.kSoftLimitForward);
    config.softLimit.reverseSoftLimit(Constants.SpinMotorConstants.kSoftLimitReverse);
    config.softLimit.forwardSoftLimitEnabled(true);
    config.softLimit.reverseSoftLimitEnabled(true);
    config.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder);

    config.inverted(false);

 
    
    config.closedLoop.pid(kpPID, kiPID, kdPID, ClosedLoopSlot.kSlot0);
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
        
        System.out.println("proceeding to setpoint = " + pGoal);
     
    }

    public void setTargetVelocity(double pGoal, boolean relative) {
        if (relative)
        {
            double curVel = getMotorVelocity();
            pGoal += curVel;
        }
        motorClosedLoop.setSetpoint(pGoal, ControlType.kVelocity, ClosedLoopSlot.kSlot1);
        motorClosedLoop.setIAccum(0);
        targetVelocity = pGoal;
        
        System.out.println("proceeding to setpoint = " + pGoal);
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


    

    public void setCurrentPosition(double meters)
    {
        motorEncoder.setPosition(meters * Constants.SpinMotorConstants.rotsPerMeter);
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
            spinMotor.stopMotor();
        }
        public void setMovement(double desiredSpeedOn)
        {
            if (motorEncoder.getVelocity() == 0)
            {
                MoveMotor(desiredSpeedOn);
            }
            else
            {
                MoveMotor(0);
            }
        }
        
        public void showData()
        {
            SmartDashboard.putNumber("motor relative pos", motorEncoder.getPosition());
        }
   
}

        