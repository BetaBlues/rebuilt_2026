package frc.robot;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.Subsystems.SwerveChassis;
import frc.robot.Subsystems.SwerveWheel;
import frc.robot.Subsystems.Motor;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Mechanism;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Config;



// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.


import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.EventImportance;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import frc.robot.Constants.ArmConstants;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants.SpinMotorConstants;
import frc.robot.Constants.ConveyorOneConstants;
import frc.robot.Constants.ConveyorTwoConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.Constants.k_chassis;

// import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.Constants.k_chassis;
//import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.k_xbox;
// import frc.robot.commands.AutonomousCommand;

import frc.robot.commands.SwerveDriveCommand;

// import swervelib.imu.NavXSwerve;

import frc.robot.Subsystems.SwerveChassis;
import frc.robot.Subsystems.SwerveWheel;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;

import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;


import static edu.wpi.first.units.Units.Rotation;

import java.util.List;
import frc.robot.commands.SwerveDriveCommand;

import frc.robot.Subsystems.ChassisGyro;
//import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;




public class RobotContainer {
    private final XboxController m_chassisController = new XboxController(0); // connect XboxController to port 0
    private final XboxController m_MechanismController = new XboxController(1); // connect XboxController to port 1
    private final ChassisGyro m_gyro = Constants.hasGyro ? new ChassisGyro(AHRS.NavXComType.kUSB1) : null;
    private final SwerveChassis m_SwerveSubsystem = Constants.hasSwerve ? new SwerveChassis(m_chassisController, m_gyro) : null;
    private final Motor m_Motor = Constants.hasMotor ? new Motor("Motor Test", 4) : null;
    private final Motor m_Launcher = Constants.hasLauncher ? new Motor("Launcher", 10) : null;
    private final Motor m_Intake = Constants.hasIntake ? new Motor("Intake", 9) : null;




    public RobotContainer() {
    // configureButtonBindings();

    if (Constants.hasSwerve) {
      DataLogManager.start();
      m_SwerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_SwerveSubsystem, m_chassisController, m_gyro));

      // new JoystickButton(m_chassisController, Constants.k_xbox.buttonRightBumper).onTrue(m_SwerveSubsystem.runOnce(m_SwerveSubsystem::zeroHeading));

      new JoystickButton(m_chassisController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_SwerveSubsystem.setWorldRotation(m_gyro.getAngle())));

         
    }


    if (Constants.hasMotor) {
      // m_Motor.setDefaultCommand(
      //   Commands.run(
      //     () ->
      //       m_Motor.runAutomatic(),
      //       m_Motor));
        //-45-> 60, -35-> 50, -25-> 40, 35-> -40
        new POVButton(m_MechanismController, 0).onTrue(new InstantCommand(()-> m_Motor.MoveMotor(SpinMotorConstants.forwardSp)));
        new POVButton(m_MechanismController, 180).onTrue(new InstantCommand(()-> m_Motor.MoveMotor(SpinMotorConstants.backwardSp)));

        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onTrue(new InstantCommand(()-> m_Motor.MovePos(0.0, false)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_Motor.MovePos(1.0, false)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(new InstantCommand(()-> m_Motor.stopMotor()));

        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onTrue(new InstantCommand(()-> m_Motor.setTargetVelocity(10.0, false)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onTrue(new InstantCommand(()-> m_Motor.setTargetVelocity(-10.0, false)));
    }
    
    if (Constants.hasLauncher) {
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onTrue(new InstantCommand(()-> m_Launcher.MoveMotor(0.3)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onTrue(new InstantCommand(()-> m_Launcher.MoveMotor(-0.3)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onFalse(new InstantCommand(()-> m_Launcher.MoveMotor(0)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onFalse(new InstantCommand(()-> m_Launcher.MoveMotor(0)));
      }

    if (Constants.hasIntake) {
        //in
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_Intake.MoveMotor(0.5)));
        //out
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onTrue(new InstantCommand(()-> m_Intake.MoveMotor(-0.5)));
        //stops
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onFalse(new InstantCommand(()-> m_Intake.MoveMotor(0.0)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onFalse(new InstantCommand(()-> m_Intake.MoveMotor(0.0)));
      }
        
    //  new JoystickButton(m_MechanismController, Constants.k_xbox.buttonLeftBumper).onTrue(Commands.runOnce(SignalLogger::start));
    //  new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(Commands.runOnce(SignalLogger::stop));

        /*
         * Joystick Y = quasistatic forward
         * Joystick A = quasistatic 
         * Joystick B = dynamic forward
         * Joystick X = dyanmic reverse
         */
      // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).whileTrue(m_Motor.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
      // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).whileTrue(m_Motor.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
      // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).whileTrue(m_Motor.sysIdDynamic(SysIdRoutine.Direction.kForward));
      // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).whileTrue(m_Motor.sysIdDynamic(SysIdRoutine.Direction.kReverse));

    }

    
      // }
}
 