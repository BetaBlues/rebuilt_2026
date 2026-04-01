package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.FollowPathCommand;
// import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import frc.robot.Subsystems.SwerveChassis;
import frc.robot.Subsystems.Motor;
import frc.robot.commands.SwerveDriveCommand;
import frc.robot.Subsystems.ChassisGyro;
import frc.robot.Subsystems.Launcher;
// import frc.robot.Subsystems.Vision;
import frc.robot.Subsystems.Climber;
import edu.wpi.first.math.geometry.Pose3d;


// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.







public class RobotContainer {
    private final XboxController m_chassisController = new XboxController(0); // connect XboxController to port 0
    private final XboxController m_MechanismController = new XboxController(1); // connect XboxController to port 1
    public final ChassisGyro m_gyro = Constants.hasGyro ? new ChassisGyro(AHRS.NavXComType.kUSB1) : null;
    private final SwerveChassis m_SwerveSubsystem; // = Constants.hasSwerve ? new SwerveChassis(m_chassisController, m_gyro, m_field) : null;
    private final Motor m_Motor = Constants.hasMotor ? new Motor("Motor Test", 6, Constants.SpinMotorConstants.pos_kP, Constants.SpinMotorConstants.pos_kI, Constants.SpinMotorConstants.pos_kD, Constants.SpinMotorConstants.feedforward,false) : null;
    private final Launcher m_Launcher = Constants.hasLauncher ? new Launcher("Launcher", 10, Constants.LauncherConstants.pos_kP, Constants.LauncherConstants.pos_kI, Constants.LauncherConstants.pos_kD, Constants.LauncherConstants.feedforward) : null;
    private final Motor m_Intake = Constants.hasIntake ? new Motor("Intake", 9, Constants.IntakeConstants.pos_kP, Constants.IntakeConstants.pos_kI, Constants.IntakeConstants.pos_kD, Constants.IntakeConstants.feedforward,false) : null;
    private final Climber m_Climber = Constants.hasClimber ? new Climber("Climber", 7, Constants.ClimberConstants.absPort, Constants.ClimberConstants.pos_kP, Constants.ClimberConstants.pos_kI, Constants.ClimberConstants.pos_kD, Constants.ClimberConstants.feedforward, true) : null;

    private Pose3d targetPose3d = new Pose3d();
    private Field2d m_field;

    private final SendableChooser<Command> autoChooser;


    public RobotContainer(Field2d field) {
     
    m_field = field;

    if (Constants.hasPathPlanner)
    {
      autoChooser = AutoBuilder.buildAutoChooser();
      SmartDashboard.putData("Auto Chooser", autoChooser);
    }
    else
    {
      autoChooser = null;
    }
  
    if (Constants.hasSwerve) {
      m_SwerveSubsystem = Constants.hasSwerve ? new SwerveChassis(m_chassisController, m_gyro, m_field) : null;
      m_SwerveSubsystem.setDefaultCommand(new SwerveDriveCommand(m_SwerveSubsystem, m_chassisController, m_gyro));
      new JoystickButton(m_chassisController, Constants.k_xbox.buttonBack).onTrue(new InstantCommand(()-> m_gyro.zeroHeading()));
      new JoystickButton(m_chassisController, Constants.k_xbox.buttonLeftBumper).onChange(new InstantCommand(()-> m_SwerveSubsystem.fieldForward(!m_chassisController.getLeftBumperButton())));
    }

    if (Constants.hasMotor) {
   
      //   new POVButton(m_MechanismController, 0).onTrue(new InstantCommand(()-> m_Motor.MoveMotor(SpinMotorConstants.forwardSp)));
      //   new POVButton(m_MechanismController, 180).onTrue(new InstantCommand(()-> m_Motor.MoveMotor(SpinMotorConstants.backwardSp)));

      //   new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onTrue(new InstantCommand(()-> m_Motor.MovePos(0.0, false)));
      //   new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_Motor.MovePos(1.0, false)));
      //  // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(new InstantCommand(()-> m_Motor.stopMotor()));

      //   new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onTrue(new InstantCommand(()-> m_Motor.setTargetVelocity(10.0, false)));
      //   new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onTrue(new InstantCommand(()-> m_Motor.setTargetVelocity(-10.0, false)));
    }
    
    if (Constants.hasLauncher) {                                                                                                            
        // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onTrue(new InstantCommand(()-> m_Launcher.MoveMotor(0.3)));
        // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_Launcher.MoveMotor(-0.3)));
        // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onFalse(new InstantCommand(()-> m_Launcher.MoveMotor(0)));
        // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onFalse(new InstantCommand(()-> m_Launcher.MoveMotor(0)));

        //new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onTrue(new InstantCommand(()-> m_Launcher.launchFuel(targetPose3d)));
         new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onTrue(new InstantCommand(()-> m_Launcher.setTargetVoltage(5))); //8
        // new POVButton(m_MechanismController, 90).onTrue(new InstantCommand(()-> m_Launcher.MoveMotor(1.0)));
         //new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onTrue(new InstantCommand(()-> m_Launcher.setTargetVelocity(5.0, false)));
        //new POVButton(m_MechanismController, 90).onFalse(new InstantCommand(()-> m_Launcher.MoveMotor(0.0)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onTrue(new InstantCommand(()-> m_Launcher.MoveMotor(-0.3)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonY).onFalse(new InstantCommand(()-> m_Launcher.MoveMotor(0)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonA).onFalse(new InstantCommand(()-> m_Launcher.MoveMotor(0)));
      }
      if (Constants.hasClimber) {
     
          // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonLeftBumper).onTrue(new InstantCommand(()-> m_Climber.setTargetPosition(Constants.ClimberConstants.setpoint1, false))); //up
          // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(new InstantCommand(()-> m_Climber.setTargetPosition(Constants.ClimberConstants.setpoint0, false))); //0 or down
          // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonLeftBumper).onTrue(new InstantCommand(()-> m_Climber.MoveToLimit(0.35))); //up?
          // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(new InstantCommand(()-> m_Climber.MoveToLimit(-0.35))); //down?
          
          // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonLeftBumper).onTrue(new InstantCommand(()-> m_Climber.MoveMotor(0.35))); //up?
          // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(new InstantCommand(()-> m_Climber.MoveMotor(-0.35))); //down?
          new JoystickButton(m_MechanismController, Constants.k_xbox.buttonLeftBumper).onTrue(new InstantCommand(()-> m_Climber.setMotorSpeed(0.35))); //up?
          new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(new InstantCommand(()-> m_Climber.setMotorSpeed(-0.35))); //down?
          new JoystickButton(m_MechanismController, Constants.k_xbox.buttonLeftBumper).onFalse(new InstantCommand(()-> m_Climber.MoveMotor(0))); 
          new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onFalse(new InstantCommand(()-> m_Climber.MoveMotor(0))); 
        }

    if (Constants.hasIntake) {
        //in
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onTrue(new InstantCommand(()-> m_Intake.MoveMotor(0.5)));
        //out
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onTrue(new InstantCommand(()-> m_Intake.MoveMotor(-0.5)));
        //stops
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onFalse(new InstantCommand(()-> m_Intake.MoveMotor(0.0)));
        new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onFalse(new InstantCommand(()-> m_Intake.MoveMotor(0.0)));

        // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonB).onChange(new InstantCommand(()-> m_Intake.setMovement(0.5)));
        // new JoystickButton(m_MechanismController, Constants.k_xbox.buttonX).onChange(new InstantCommand(()-> m_Intake.setMovement(-0.5)));
      }
        
    // if (Constants.hasClimber) {
    //     00new JoystickButton(m_MechanismController, Constants.k_xbox.buttonLeftBumper).onTrue(new InstantCommand(()-> m_Climber.MovePos(0.0)));
    //     new JoystickButton(m_MechanismController, Constants.k_xbox.buttonRightBumper).onTrue(new InstantCommand(()-> m_Climber.MovePos(-70.0)));
    //     new POVButton(m_MechanismController, 180).onTrue(new InstantCommand(()-> m_Climber.MovePos(70.0)));
       
    //   }


   
    }

    
    
    public Command getAutonomousCommand() {
      try {
        //switch to this eventually
          //return autoChooser.getSelected();


          // Load the path you want to follow using its name in the GUI
        PathPlannerPath path = PathPlannerPath.fromPathFile("Example Path");

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        return AutoBuilder.followPath(path);
      }
      catch (Exception e) {
        DriverStation.reportError("Big oops: " + e.getMessage(), e.getStackTrace());
        return Commands.none();
      }
    }



  public void showData()
  {
    if (m_Climber != null) // checks if null
    {
      m_Climber.showData();
    }
    // if (m_SwerveSubsystem != null)
    // {
    //  // m_SwerveSubsystem.showData();
    // }
  }
}
 
