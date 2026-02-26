package frc.robot;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Subsystems.SwerveChassis;
import frc.robot.Subsystems.Motor;
import frc.robot.Constants.SpinMotorConstants;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.Subsystems.ChassisGyro;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;



// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.







public class RobotContainer {
    private final XboxController m_chassisController = new XboxController(0); // connect XboxController to port 0
    private final XboxController m_MechanismController = new XboxController(1); // connect XboxController to port 1
    public final ChassisGyro m_gyro = Constants.hasGyro ? new ChassisGyro(AHRS.NavXComType.kUSB1) : null;
    private final SwerveChassis m_SwerveSubsystem = Constants.hasSwerve ? new SwerveChassis(m_chassisController, m_gyro) : null;
    private final Motor m_Motor = Constants.hasMotor ? new Motor("Motor Test", 4) : null;
    private final Motor m_Launcher = Constants.hasLauncher ? new Motor("Launcher", 10) : null;
    private final Motor m_Intake = Constants.hasIntake ? new Motor("Intake", 9) : null;
    private final Motor m_Climber = Constants.hasClimber ? new Motor("Climber", 7) : null;




    public RobotContainer() {
  

    if (Constants.hasSwerve) {
      DataLogManager.start();
      m_SwerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_SwerveSubsystem, m_chassisController, m_gyro));

      
      new JoystickButton(m_chassisController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_SwerveSubsystem.setWorldRotation(m_gyro.getAngle())));

        /*
          * Swerve Drive Kinematics and Odometry Setup
          * 
          * The kinematics object allows us to convert between chassis speeds and individual wheel speeds.
          * The odometry object allows us to track the robot's position on the field over time using the kinematics and sensor data.
          * 
          * Below, we define the locations of our swerve modules relative to the robot center, create our kinematics object from those locations,
          * and then create our odometry object from the kinematics and our initial wheel positions.
          */
        // Locations for the swerve drive modules relative to the robot center.
        Translation2d m_frontLeftLocation = new Translation2d(0.381, 0.381);
        Translation2d m_frontRightLocation = new Translation2d(0.381, -0.381);
        Translation2d m_backLeftLocation = new Translation2d(-0.381, 0.381);
        Translation2d m_backRightLocation = new Translation2d(-0.381, -0.381);
        // Creating my kinematics object using the module locations
        SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
          m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
        );
      

  
   




    if (Constants.hasMotor) {
   
        new POVButton(m_MechanismController, 0).onTrue(new InstantCommand(()-> m_Motor.MoveMotor(SpinMotorConstants.forwardSp)));
        new POVButton(m_MechanismController, 180).onTrue(new InstantCommand(()-> m_Motor.MoveMotor(SpinMotorConstants.backwardSp)));

        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onTrue(new InstantCommand(()-> m_Motor.MovePos(0.0, false)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_Motor.MovePos(1.0, false)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(new InstantCommand(()-> m_Motor.stopMotor()));

        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onTrue(new InstantCommand(()-> m_Motor.setTargetVelocity(10.0, false)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onTrue(new InstantCommand(()-> m_Motor.setTargetVelocity(-10.0, false)));
    }
    
    if (Constants.hasLauncher) {
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onTrue(new InstantCommand(()-> m_Launcher.MoveMotor(0.3)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_Launcher.MoveMotor(-0.3)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onFalse(new InstantCommand(()-> m_Launcher.MoveMotor(0)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onFalse(new InstantCommand(()-> m_Launcher.MoveMotor(0)));
      }
      if (Constants.hasClimber) {
      m_Climber.setDefaultCommand(
      Commands.run(
        () ->
          m_Climber.runAutomatic(),
          m_Climber));
          new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_Climber.MoveArm(Constants.ClimberConstants.setpoint1)));
          new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onTrue(new InstantCommand(()-> m_Climber.MoveArm(Constants.ClimberConstants.setpoint0)));
      }

    if (Constants.hasIntake) {
        //in
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onTrue(new InstantCommand(()-> m_Intake.MoveMotor(0.5)));
        //out
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onTrue(new InstantCommand(()-> m_Intake.MoveMotor(-0.5)));
        //stops
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onFalse(new InstantCommand(()-> m_Intake.MoveMotor(0.0)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onFalse(new InstantCommand(()-> m_Intake.MoveMotor(0.0)));
      }
        
    // if (Constants.hasClimber) {
    //     new JoystickButton(m_MechanismController, Constants.k_xbox.buttonLeftBumper).onTrue(new InstantCommand(()-> m_Climber.MovePos(0.0)));
    //     new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(new InstantCommand(()-> m_Climber.MovePos(-70.0)));
    //     new POVButton(m_MechanismController, 180).onTrue(new InstantCommand(()-> m_Climber.MovePos(70.0)));
       
    //   }
   
    }

}
}
 