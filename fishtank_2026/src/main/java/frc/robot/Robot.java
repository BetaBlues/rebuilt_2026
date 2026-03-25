package frc.robot;

import edu.wpi.first.wpilibj.DataLogManager;
import com.pathplanner.lib.commands.FollowPathCommand;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoSink;
import frc.robot.Subsystems.Vision;

// import edu.wpi.first.math.geometry.Transform3d;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.networktables.NetworkTableInstance;
// import edu.wpi.first.networktables.StructArrayPublisher;
// import edu.wpi.first.networktables.StructPublisher;
// import edu.wpi.first.wpilibj.DataLogManager;
// import edu.wpi.first.wpilibj.PneumaticsModuleType;
// import edu.wpi.first.wpilibj.Timer;
// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the manifest
 * file in the resource
 * directory.
 */
public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  private static final String kDefaultAuto = "Default";
  private static final String kLeftAuto = "LeftAuto";
  private static final String kMiddleRightAuto = "MiddleRightAuto";
  private static final String kMiddleLeftAuto = "MiddleLeftAuto";
  private static final String kRightAuto = "RightAuto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();

  private Field2d m_field = new Field2d();

  private UsbCamera intakeCamera;
  private UsbCamera climberCamera;
  VideoSink server;

  private Vision m_vision;
  private Pose3d m_pose;

  private RobotContainer m_robotContainer;
  private PowerDistribution pdh;

  XboxController m_MechanismController = new XboxController(1);

  // StructPublisher<Pose3d> publisher =
  // NetworkTableInstance.getDefault().getStructTopic("MyPose",
  // Pose3d.struct).publish();
  // StructArrayPublisher<Pose3d> arrayPublisher =
  // NetworkTableInstance.getDefault().getStructArrayTopic("MyPoseArray",
  // Pose3d.struct).publish();

  public Robot() {
    DataLogManager.start();
    m_autoSelected = kDefaultAuto;
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("Left Auto", kLeftAuto);
    m_chooser.addOption("MiddleRight Auto", kMiddleRightAuto);
    m_chooser.addOption("MiddleLeft Auto", kMiddleLeftAuto);
    m_chooser.addOption("Right Auto", kRightAuto);
    // SmartDashboard.putData("Auto choices", m_chooser);

    pdh = new PowerDistribution();
    pdh.clearStickyFaults();

    if (Constants.hasUsbCameras) {
      intakeCamera = CameraServer.startAutomaticCapture("Intake Camera", 0);
      climberCamera = CameraServer.startAutomaticCapture("Climber Camera", 1);
      server = CameraServer.getServer();
    }
    if (Constants.hasVision) {
      m_vision = new Vision();
    }

  }

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */

  public void periodic() {
    // This will get the simulated sensor readings that we set
    // in the previous article while in simulation, but will use
    // real values on the robot itself.

  }

  @Override
  public void robotInit() {
    DataLogManager.start();
    m_field.setRobotPose(1.0, 1.0, Rotation2d.fromDegrees(0.0));
    // SmartDashboard.putData("Field", m_field);
    m_robotContainer = new RobotContainer(m_field);

    // pdh = new PowerDistribution();
    //pdh.clearStickyFaults();

    if (Constants.hasPathPlanner) {
      // for pathplanning to decrease delay
      FollowPathCommand.warmupCommand().schedule();
    }
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {

    // Runs the Scheduler. This is responsible for polling buttons, adding
    // newly-scheduled
    // commands, running already-scheduled commands, removing finished or
    // interrupted commands,
    // and running subsystem periodic() methods. This must be called from the
    // robot's periodic
    // block in order for anything in the Command-based framework to work.

    CommandScheduler.getInstance().run();
    // Let scheduler run first before showing debug data.
    m_robotContainer.showData();

  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {

    // This method loads the auto when it is called, however, it is recommended
    // to first load your paths/autos when code starts, then return the
    // pre-loaded auto/path

    m_autoSelected = m_chooser.getSelected();

    System.out.println("Auto selected: " + m_autoSelected);

    // should find where it currently is, where it should be,
    // how far it is from that position and move there to shoot
    // mix of vision and odo??
    //
    switch (m_autoSelected) {
      case kLeftAuto:
        // m_leftCamera.getPose3d().getTranslation();
        break;

      case kMiddleRightAuto:

        break;

      case kMiddleLeftAuto:

        break;

      case kRightAuto:

        break;

      case kDefaultAuto:
      default:

        break;
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {

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

  }

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
