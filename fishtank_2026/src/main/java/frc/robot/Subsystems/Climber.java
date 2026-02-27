package frc.robot.Subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Subsystems.Motor;

public class Climber extends Motor{
    private SparkMax spinMotor;
    private RelativeEncoder motorEncoder;
    private SparkMaxConfig config;
    private double m_setpoint = 0.0; //was 70.0
    private boolean m_Active = false;
    private PIDController spinMotorPID;
    private SysIdRoutine routine;
    private SparkClosedLoopController motorClosedLoop;
    private double targetVelocity;
    private double targetPosition;
    private PIDController climberPID;
    private double m_eSetpoint = 0.0;
    private String loc;

    private SysIdRoutine log;
    public Climber (String name, int canId)
    {
        super(name, canId);
    }

    
        public double getTurningPosition() {
            return motorEncoder.getPosition();
        }

          public void setTargetPosition(double pGoal) {
        double curPos = getClimberPosition();
        if (curPos != pGoal) {
            m_eSetpoint = pGoal;
            m_Active = true; // this needs to be true to enable the motor
            System.out.println("proceeding to setpoint = " + pGoal);
        }
    }

    /**
     * Drives the arm to a position using a trapezoidal motion profile. This
     * function is usually
     * wrapped in a {@code RunCommand} which runs it repeatedly while the command is
     * active.
     *
     * <p>
     * This function updates the motor position control loop using a setpoint from
     * the trapezoidal
     * motion profile. The target position is the last set position with
     * {@code setTargetPosition}.
     */
    public void runAutomatic() {
        double rotationSpeed = -1.0*climberPID.calculate(getClimberPosition(), m_eSetpoint);
        if (rotationSpeed > Constants.ClimberConstants.MaxRotationSpeed) { 
            rotationSpeed= Constants.ClimberConstants.MaxRotationSpeed;
        }
        else if (rotationSpeed < -Constants.ClimberConstants.MaxRotationSpeed){
            rotationSpeed = -Constants.ClimberConstants.MaxRotationSpeed;
        }
        SmartDashboard.putNumber("Climber height", getClimberPosition());
        SmartDashboard.putNumber("climberTarget", m_eSetpoint);
        SmartDashboard.putNumber("desired speed", rotationSpeed);
        SmartDashboard.putBoolean("Active", m_Active);
        SmartDashboard.putNumber(loc+"-encoderVal", getTurningPosition() % 360);
        
        if (m_Active) {
            
            if (Math.abs(rotationSpeed) > 0.0005) {
                spinMotor.set(rotationSpeed);
            } else {
                stopMotor();
              
            }
        }
    }

    public double getClimberPosition() {
        return -1.0*motorEncoder.getPosition()+ Constants.ClimberConstants.ClimberOffset;
        //replace elevatorEncoder with absolute encoder
    }

    // turns the arm
    public void MoveArm(double rotation) {
        setTargetPosition(rotation);
    }
}
