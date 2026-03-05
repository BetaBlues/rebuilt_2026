package frc.robot.Subsystems;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Subsystems.Motor;

public class Climber extends Motor{
    private DutyCycleEncoder absoluteEncoder;
    private double curPos;
    private PIDController climberPID;

    private SysIdRoutine log;
    public Climber (String name, int canId, int encoderPort, double kpPID, double kiPID, double kdPID)
    {
        super(name, canId, kpPID, kiPID, kdPID);
        absoluteEncoder = new DutyCycleEncoder(encoderPort);
        curPos = absoluteEncoder.get();
        curPos *= Constants.ClimberConstants.ticsPerMeters; //translating to meters
        curPos -= Constants.ClimberConstants.startPos; //where it turns on
        climberPID.enableContinuousInput(-180, 180);
    }

    public void showData()
    {
        SmartDashboard.putNumber("Climber height", absoluteEncoder.get());
        super.showData();
        
    }
}
