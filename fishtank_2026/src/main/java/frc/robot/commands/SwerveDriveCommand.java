package frc.robot.commands;

import frc.robot.Constants;
import frc.robot.Constants.k_chassis;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.SwerveChassis;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.ChassisGyro;


public class SwerveDriveCommand extends Command {
    public final SwerveChassis m_chassis;
    public final XboxController m_controller; 
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    private final ChassisGyro m_gyro;
    private double currentAngle;
    
    
    
    public SwerveDriveCommand(SwerveChassis chassis, XboxController controller, ChassisGyro gyro){
        this.m_chassis = chassis; 
        this.m_controller = controller;
        this.xLimiter = new SlewRateLimiter(k_chassis.AccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(k_chassis.AccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(k_chassis.AngularAccelerationUnitsPerSecond);
        this.m_gyro = gyro;
        chassis.setWorldRotation(0.0);
        addRequirements(m_chassis);
    }

   

     @Override
     public void execute() {
    //    currentAngle = Math.toRadians(m_gyro.getCompassHeading());

       
        // double temp = m_controller.getLeftX() * Math.cos(currentAngle) + m_controller.getLeftY() * Math.sin(currentAngle);

        // double ySpeedSquared = Math.pow(m_controller.getLeftX() - temp*Math.cos(currentAngle), 2) + Math.pow((m_controller.getLeftY() -temp*Math.sin(currentAngle)), 2); //m_controller.getLeftY();
        // double ySpeed = ySpeedSquared > 0 ? Math.sqrt(ySpeedSquared) : - Math.sqrt(ySpeedSquared);


        
        // double xSpeedSquared = Math.pow(temp*Math.cos(currentAngle), 2) + Math.pow(temp*Math.sin(currentAngle), 2);
        // double xSpeed = xSpeedSquared > 0 ? Math.sqrt(xSpeedSquared) : - Math.sqrt(xSpeedSquared);
        // double turningSpeed = m_controller.getRightX();

        // double vec1 = Math.abs(ySpeed) * Math.abs(xSpeed) * Math.cos(currentAngle);

        // double xVal = m_controller.getLeftY();
        
        // double yVal;
        // if (Constants.hasCanCoder)
        // {
        //     yVal = m_controller.getLeftX(); /*-1 */
        // }
        // else
        // {
        //     yVal = m_controller.getLeftX();
        // }

        
        // double cos_w = Math.cos(Math.toRadians(m_chassis.getWorldRotation()));
        // double sin_w = Math.sin(Math.toRadians(m_chassis.getWorldRotation()));

        // double rotatedXInput = xVal * cos_w - yVal * sin_w;
        // double rotatedYInput = xVal * sin_w + yVal * cos_w;

        // // double rotatedXInput = xVal;
        // // double rotatedYInput = yVal;


        // double xSpeed = rotatedXInput;
        // double ySpeed = rotatedYInput;
        // double turningSpeed = m_controller.getRightX();

        // xSpeed = Math.abs(xSpeed) > k_chassis.kDeadband ? xSpeed : 0.0;
        // ySpeed = Math.abs(ySpeed) > k_chassis.kDeadband ? ySpeed : 0.0;
        // turningSpeed = Math.abs(turningSpeed) > k_chassis.kDeadband ? turningSpeed : 0.0;
        double joystickForward = -m_controller.getLeftY();
        double joystickStrafe = m_controller.getLeftX();
        double joystickTurn = m_controller.getRightX();

        joystickForward = Math.abs(joystickForward) > k_chassis.kDeadband ? joystickForward : 0.0;
        joystickStrafe = Math.abs(joystickStrafe) > k_chassis.kDeadband ? joystickStrafe : 0.0;
        joystickTurn = Math.abs(joystickTurn) > k_chassis.kDeadband ? joystickTurn : 0.0;
        


        // //makes driving smoother
        // xSpeed = xLimiter.calculate(xSpeed) * k_chassis.MaxMetersPerSecond;
        // ySpeed = yLimiter.calculate(ySpeed) * k_chassis.MaxMetersPerSecond;
        // turningSpeed = turningLimiter.calculate(turningSpeed) * k_chassis.AngularAccelerationUnitsPerSecond;

        double xSpeed = xLimiter.calculate(joystickForward) * k_chassis.MaxMetersPerSecond;
        double ySpeed = yLimiter.calculate(joystickStrafe) * k_chassis.MaxMetersPerSecond;
        double turningSpeed = turningLimiter.calculate(joystickTurn) * k_chassis.kPhysicalMaxAngularSpeedRadiansPerSecond;
        
        

        SmartDashboard.putNumber("xSpeed mod", xSpeed);
        SmartDashboard.putNumber("ySpeed mod", ySpeed);
        SmartDashboard.putNumber("turningSpeed mod", turningSpeed);

        // The gyro is causing the wheel drift, need to test if x, y and turning are zero, if so, set chassis speed to zero
        // and do not call fromFieldRelativeSpeeds
        
        ChassisSpeeds chassisSpeeds;

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, m_chassis.getRotation2d());
        SwerveModuleState[] moduleStates = m_chassis.driveKinematics().toSwerveModuleStates(chassisSpeeds);
        
        
        // Need to replace this with kinematics calculations
        //
        if (Constants.testMotors) {
            m_chassis.testMotors(ySpeed, turningSpeed);
        } else {
            m_chassis.setModuleStates(moduleStates, true);
        }
        

        // SwerveModuleState[] moduleStates = kinematics.toSwerveModuleStates(speeds);
        // m_chassis.setModuleStates(states);
     }
    
     @Override
     public void end(boolean interrupted) {
        m_chassis.stopModules();
     }
     
     @Override 
     public boolean isFinished() {
        return false;
     }
}
