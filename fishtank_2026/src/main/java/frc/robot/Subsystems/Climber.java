package frc.robot.Subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.motorcontrol.PWMVictorSPX;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Subsystems.Motor;

public class Climber extends Motor{
    private DutyCycleEncoder absoluteEncoder;
    private double curPos;
    private PIDController climberPID;
    DigitalInput m_bottomlimitSwitch = new DigitalInput(2);
    private boolean m_lowerLimit = false;
  

    private SysIdRoutine log;
    public Climber (String name, int canId, int encoderPort, double kpPID, double kiPID, double kdPID, double feedforward, boolean continuous)
    {
        super(name, canId, kpPID, kiPID, kdPID, feedforward, continuous);
        absoluteEncoder = new DutyCycleEncoder(encoderPort, 1, 0);
        curPos = absoluteEncoder.get();
        curPos *= Constants.ClimberConstants.ticsPerMeters; //translating to meters
        curPos -= Constants.ClimberConstants.startPos; //where it turns on

        

    }

    public void MoveToLimit(double speed) {
        double position = absoluteEncoder.get();
        SmartDashboard.putNumber("Move abs get", position);
        SmartDashboard.putNumber("Move speed", speed);
        if ((speed > 0) && (position < Constants.ClimberConstants.Max_Height))
        {
        SmartDashboard.putNumber("Move it now", speed);
            MoveMotor(speed);
        } 
        else if ((speed < 0) && (position > Constants.ClimberConstants.Min_Height))
        {
        SmartDashboard.putNumber("Move it now", speed);
            MoveMotor(speed);
        }
        else
        {
            MoveMotor(0.0);
        }
       
    }
    
    
  
  

  /**
   * Sets the motor speed based on joystick input while respecting limit switches.
   *
   * @param speed the desired speed of the motor, positive for up and negative for down
   */
  public void setMotorSpeed(double speed) {
   
    
    if (speed < 0 && m_lowerLimit) 
    {
        MoveMotor(0);

    } 
    else 
    {
        
        MoveMotor(speed);
    }
    
  }

    public void showData()
    {
        SmartDashboard.putNumber("Climber height", absoluteEncoder.get());
        SmartDashboard.putNumber("relative climb height", getRelativePosition());
        SmartDashboard.putBoolean("limit switch", m_bottomlimitSwitch.get());
        super.showData();
        
    }

    @Override
    public void periodic() {
        m_lowerLimit = m_bottomlimitSwitch.get();
        if (m_lowerLimit) {
            MoveMotor(0);
        }
    }
}
