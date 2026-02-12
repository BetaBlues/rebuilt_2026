package frc.robot;


import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cameraserver.CameraServerShared;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Subsystems.SwerveChassis;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Subsystems.Vision;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  // private SwerveChassis swerveChassis; 
  private Command m_autonomousCommand;
  private static final String kDefaultAuto = "Default";
  private static final String kLeftAuto = "LeftAuto";
  private static final String kMiddleAuto = "MiddleAuto";
  private static final String kRightAuto = "Right Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Field2d m_field = new Field2d();
  private Vision m_vision = new Vision();
  

  //simulation testing
  // private XboxController m_SwerveController = new XboxController(0);
  // private double x = 1.0;
  // private double y = 1.0;
  // private double heading = 0.0;
  // private static final double MAX_SPEED = 1.5;
  // private static final double MAX_ROT = Math.PI;

  private RobotContainer m_robotContainer;
  private PowerDistribution pdh;

  XboxController m_MechanismController = new XboxController(1);
  


//  private IntakeSubsystem m_IntakeSubsystem;

  PneumaticsModuleType ctrepcm = PneumaticsModuleType.CTREPCM;

  private final Timer m_timer = new Timer();
  // UsbCamera seaSawCamera;
  // UsbCamera driveCamera;
  // UsbCamera pistonCamera;
  // VideoSink server;

  public Robot() {

    m_autoSelected = kDefaultAuto;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Left Auto", kLeftAuto);
    m_chooser.addOption("Middle Auto", kMiddleAuto);
    m_chooser.addOption("Right Auto", kRightAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    // seaSawCamera = CameraServer.startAutomaticCapture("SeeSaw Camera", 0);
    // driveCamera = CameraServer.startAutomaticCapture("Drive Camera", 1);
    // pistonCamera = CameraServer.startAutomaticCapture("Piston Camera", 2);
    // server = CameraServer.getServer();
    
  }

  /**
   * Uses the CameraServer class to automatically capture video from a USB webcam and send it to the
   * FRC dashboard without doing any vision processing. This is the easiest way to get camera images
   * to the dashboard. Just add this to the robotInit() method in your program.
   */

  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  
  public void periodic() {
      // This will get the simulated sensor readings that we set
      // in the previous article while in simulation, but will use
      // real values on the robot itself.
      
      
      
  }
  @Override
  public void robotInit() {
    SmartDashboard.putData("Field", m_field);
    //SmartDashboard.putData("adlkfbsda", m_field);
    // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
    // autonomous chooser on the dashboard.
    m_robotContainer = new RobotContainer();
    //IntakeSubsystem = new IntakeSubsystem();
     

    //extendArmCommand = new ExtendArmCommand(armSubsystem);
    

    pdh = new PowerDistribution();
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
    m_vision.estimatePose(m_field, m_robotContainer.m_gyro);
    m_vision.showData();
    // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
    // commands, running already-scheduled commands, removing finished or interrupted commands,
    // and running subsystem periodic() methods.  This must be called from the robot's periodic
    // block in order for anything in the Command-based framework to work.
    // m_robotContainer.updateDashboard();
    // double t = Timer.getFPGATimestamp();
    // field.setRobotPose(
    //   new Pose2d(t * 0.1, 1.0, new Rotation2d())
    // );

    // Pose2d robotPose = new Pose2d(
    //   1.0,
    //   2.0,
    //   new Rotation2d()
    // );
    // field.setRobotPose(robotPose);

    CommandScheduler.getInstance().run();

    // SmartDashboard.putNumber("Front Left Voltage", m_robotContainer.m_SwerveSubsystem.leftFrontMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Front Right Voltage", m_robotContainer.m_SwerveSubsystem.rightFrontMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Back Left Voltage", m_robotContainer.m_SwerveSubsystem.leftRearMotor.getOutputCurrent());
    // SmartDashboard.putNumber("Back Right Voltage", m_robotContainer.m_SwerveSubsystem.rightRearMotor.getOutputCurrent());

    // SmartDashboard.putNumber("Voltage", pdh.getVoltage());
    // SmartDashboard.putNumber("Gyro Direction", m_robotContainer.gyro.getAngle());
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  /** This autonomous runs the autonomous command selected by your {@link RobotContainer} class. */
  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
        // Use AutoBuilder to load the path from PathPlanner for when we have kinematics 
        //TODO: add autoconstants 
  /*       AutoBuilder autoBuilder = new AutoBuilder(
            swerveChassis::getPose,               // A method to get robot pose (use your swerve subsystem)
            swerveChassis::resetOdometry,         // A method to reset the robot's odometry
            swerveChassis.getKinematics(),        // Swerve drive kinematics
            AutoConstants.getRamseteController(), // Ramsete controller settings
            swerveChassis::drive,                 // Method to drive the swerve modules
            new PathPlannerAuto()     ); */
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    
    //  m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    // // schedule the autonomous command (example)
    // if (m_autonomousCommand != null) {
    //   m_autonomousCommand.schedule();
    //  }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    // switch (m_autoSelected) {
    //     case kLeftAuto:
    //     System.out.println("auto kLeftAuto selected");
    //     // swerveChassis.zeroHeading();
    //     // swerveChassis.driveTime(2,0.7,180.0);
    //     break;

    //     case kMiddleAuto:
    //     System.out.println("auto kMiddleAuto selected");
    //     // swerveChassis.zeroHeading();
    //     // swerveChassis.driveTime(2,0.7,0.0);
    //     break;

    //   case kRightAuto:
    //   System.out.println("auto kRightAuto selected");
    //     // swerveChassis.zeroHeading();
    //     // swerveChassis.driveTime(2,0.7,90.0);
    //     break;

    //   case kDefaultAuto:
    //   default:
    //   System.out.println("auto kDefaultAuto selected");
    //     // swerveChassis.zeroHeading();
    //     // swerveChassis.driveTime(2,0.7,0.0);
    //     break;
    // }
  }

  

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
   
    //added to test sim movement without swerve code
    // double forward =- m_SwerveController.getLeftY();
    // double turn = m_SwerveController.getRightX();

    // double dt = 0.02;

    // heading += turn * MAX_ROT * dt;
    // x += forward * MAX_SPEED * Math.cos(heading) * dt;
    // y += forward * MAX_SPEED * Math.sin(heading) * dt;
    // field.setRobotPose(new Pose2d(x, y, new Rotation2d(heading)));

    // if (m_MechanismController.getBButtonPressed()) {
    //   System.out.println("Setting drive camera");
    //   server.setSource(driveCamera);
    // }

    // else if (m_MechanismController.getBButtonReleased()) {
    //   System.out.println("Setting seesaw camera");
    //   server.setSource(seaSawCamera);
    // }
  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {} 
}
