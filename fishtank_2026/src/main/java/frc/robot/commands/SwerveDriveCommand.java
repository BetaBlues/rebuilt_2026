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

public class SwerveDriveCommand extends Command {
    public final SwerveChassis m_chassis;
    public final XboxController m_controller; 
    private SlewRateLimiter xLimiter, yLimiter, turningLimiter;
    
    public SwerveDriveCommand(SwerveChassis chassis, XboxController controller){
        this.m_chassis = chassis; 
        this.m_controller = controller;
        this.xLimiter = new SlewRateLimiter(k_chassis.AccelerationUnitsPerSecond);
        this.yLimiter = new SlewRateLimiter(k_chassis.AccelerationUnitsPerSecond);
        this.turningLimiter = new SlewRateLimiter(k_chassis.AngularAccelerationUnitsPerSecond);
        addRequirements(m_chassis);
    }

     @Override
     public void execute() {
        double ySpeed = m_controller.getLeftY();
        double xSpeed = m_controller.getLeftX();
        double turningSpeed = m_controller.getRightX();

        SmartDashboard.putNumber("xSpeed", xSpeed);
        SmartDashboard.putNumber("ySpeed", ySpeed);
        SmartDashboard.putNumber("turningSpeed", turningSpeed);

        xSpeed = Math.abs(xSpeed) > k_chassis.kDeadband ? xSpeed : 0.0;
        ySpeed = Math.abs(ySpeed) > k_chassis.kDeadband ? ySpeed : 0.0;
        turningSpeed = Math.abs(turningSpeed) > k_chassis.kDeadband ? turningSpeed : 0.0;

        //makes driving smoother
        xSpeed = xLimiter.calculate(xSpeed) * k_chassis.MaxMetersPerSecond;
        ySpeed = yLimiter.calculate(ySpeed) * k_chassis.MaxMetersPerSecond;
        turningSpeed = turningLimiter.calculate(turningSpeed) * k_chassis.AngularAccelerationUnitsPerSecond;

        SmartDashboard.putNumber("xSpeed mod", xSpeed);
        SmartDashboard.putNumber("ySpeed mod", ySpeed);
        SmartDashboard.putNumber("turningSpeed mod", turningSpeed);

        // The gyro is causing the wheel drift, need to test if x, y and turning are zero, if so, set chassis speed to zero
        // and do not call fromFieldRelativeSpeeds
        
        ChassisSpeeds chassisSpeeds;

        chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                    xSpeed, ySpeed, turningSpeed, m_chassis.getRotation2d());
        SwerveModuleState[] moduleStates = k_chassis.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
        
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
