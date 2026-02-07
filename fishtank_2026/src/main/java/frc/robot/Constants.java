package frc.robot;

// import edu.wpi.first.math.controller.ArmFeedforward;
// import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Constants {
    public final static boolean hasGyro = true;
    public final static boolean hasSwerve = true;
    public final static boolean hasSeesaw = false;
    public final static boolean hasMotor = false;
    public final static boolean hasConveyorOne = false;
    public final static boolean hasConveyorTwo = false;
    public final static boolean hasIntakeRotation = false;
    public final static boolean hasIntakeWheels = false;
    public final static boolean hasLauncher = false;
    public static boolean testMotors = false;

    public static final int rightFrontAbsEncPort = 13, leftFrontAbsEncPort = 23, leftRearAbsEncPort = 33, rightRearAbsEncPort = 43; //1,2,3,4 in order
    public static final double leftFrontAbsOffset =  0;//1.47;
    public static final double rightFrontAbsOffset = 0;//2.19;
    public static final double leftRearAbsOffset = 0;//-1.57;
    public static final double rightRearAbsOffset= 0;//2.145;
    public static final double steerEncoderRatio = 3.406;
    public static final double driveEncoderRatio = 10.0;


    public static class SeeSawConstants {
      //gear box is 1:10
      public static final int kCanId = 5;
      public static final int kCurrentLimit = 40;
      public static final double MaxRotationSpeed= 0.18; //change to a lower speed to make smoother //0.04
     
    

      // public static final PIDGains kPIDGains = new PIDGains(0.4, 0.0, 0.0);

      public static final double kVelocityInput = -0.05; //edit
      public static final double kVelocityOutput = 0.05; //edit
      public static final double kPositionInput = -1.17;
      public static final double kPositionOutput = 0;
      public static final double kPSSRotation = 0.01;
      public static final double SeeSawOffset = -20.0; //70.0

      public static final double kSoftLimitReverse = -25000;
      public static final double kSoftLimitForward = 35000;

    }
    
    public static class ConveyorOneConstants {
      //gear box is 1:10
      public static final int kCanId = 6;
      public static final int kCurrentLimit = 40;
      public static final double MaxRotationSpeed= 0.04; //change to a lower speed to make smoother //0.04
     
    

      // public static final PIDGains kPIDGains = new PIDGains(0.4, 0.0, 0.0);

      public static final double forwardSp = -0.6;
      public static final double backwardSp = 0.6;
      public static final double kPositionInput = -1.17;
      public static final double kPositionOutput = 0;
      public static final double pos_kP = 0.4;
      public static final double pos_kI = 0.0;
      public static final double pos_kD = 0.0;
      public static final double vel_kP = 1.0;
      public static final double vel_kI = 0.0;
      public static final double vel_kD = 0.05;
      public static final double MotorOffset = 0;//-20.0; //70.0

      public static final double kSoftLimitReverse = -25000;
      public static final double kSoftLimitForward = 35000;

    }
    public static class ConveyorTwoConstants {
      //gear box is 1:10
      public static final int kCanId = 7;
      public static final int kCurrentLimit = 40;
      public static final double MaxRotationSpeed= 0.04; //change to a lower speed to make smoother //0.04
     
    

      // public static final PIDGains kPIDGains = new PIDGains(0.4, 0.0, 0.0);

      public static final double forwardSp = -0.6;
      public static final double backwardSp = 0.6;
      public static final double kPositionInput = -1.17;
      public static final double kPositionOutput = 0;
      public static final double pos_kP = 0.4;
      public static final double pos_kI = 0.0;
      public static final double pos_kD = 0.0;
      public static final double vel_kP = 1.0;
      public static final double vel_kI = 0.0;
      public static final double vel_kD = 0.05;
      public static final double MotorOffset = 0;//-20.0; //70.0

      public static final double kSoftLimitReverse = -25000;
      public static final double kSoftLimitForward = 35000;

    }
    public static class IntakeConstants {
      //gear box is 1:10
      public static final int kCanId = 9;
      public static final int kCurrentLimit = 60;
      public static final double MaxRotationSpeed= 0.04; //change to a lower speed to make smoother //0.04
     
    

      // public static final PIDGains kPIDGains = new PIDGains(0.4, 0.0, 0.0);

      public static final double forwardSp = -0.6;
      public static final double backwardSp = 0.6;
      public static final double kPositionInput = -1.17;
      public static final double kPositionOutput = 0;
      public static final double pos_kP = 0.4;
      public static final double pos_kI = 0.0;
      public static final double pos_kD = 0.0;
      public static final double vel_kP = 1.0;
      public static final double vel_kI = 0.0;
      public static final double vel_kD = 0.05;
      public static final double IntakeMotorOffset = 0;//-20.0; //70.0

      public static final double kSoftLimitReverse = -25000;
      public static final double kSoftLimitForward = 35000;

    }
    public static class IntakeRotationConstants {
      //gear box is 1:10
      public static final int kCanId = 10;
      public static final int kCurrentLimit = 40;
      public static final double MaxRotationSpeed= 0.02; //change to a lower speed to make smoother //0.04
     
    

      // public static final PIDGains kPIDGains = new PIDGains(0.4, 0.0, 0.0);

      public static final double forwardSp = 0.2;
      public static final double backwardSp = -0.2;
      public static final double kPositionInput = -1.17;
      public static final double kPositionOutput = 0;
      public static final double pos_kP = 0.04;
      public static final double pos_kI = 0.0;
      public static final double pos_kD = 0.0;
      public static final double vel_kP = 1.0;
      public static final double vel_kI = 0.0;
      public static final double vel_kD = 0.05;
      public static final double MotorOffset = 0;//-20.0; //70.0

      public static final double kSoftLimitReverse = -25000;
      public static final double kSoftLimitForward = 35000;

    }

    public static class SpinMotorConstants {
      //gear box is 1:10
      public static final int kCanId = 9;
      public static final int kCurrentLimit = 40;
      public static final double MaxRotationSpeed= 0.04; //change to a lower speed to make smoother //0.04
     
    

      // public static final PIDGains kPIDGains = new PIDGains(0.4, 0.0, 0.0);

      public static final double forwardSp = 0.2;
      public static final double backwardSp = -0.2;
      public static final double kPositionInput = -1.17;
      public static final double kPositionOutput = 0;
      public static final double pos_kP = 0.4;
      public static final double pos_kI = 0.0;
      public static final double pos_kD = 0.0;
      public static final double vel_kP = 1.0;
      public static final double vel_kI = 0.0;
      public static final double vel_kD = 0.05;
      public static final double MotorOffset = 0;//-20.0; //70.0

      public static final double kSoftLimitReverse = -25000;
      public static final double kSoftLimitForward = 35000;

    }

    


    public static class OperatorConstants {
        public static final int kDriverControllerPort = 0;
      }

    public final class k_xbox {
    //buttons --> correct
        public static final int buttonA = 1; 
        public static final int buttonB = 2;
        public static final int buttonX = 3;
        public static final int buttonY = 4;
        public static final int buttonLeftBumper = 5;
        public static final int buttonRightBumper = 6;
        public static final int buttonLeftLowerBumper = 9;
        public static final int buttonRightLowerBumper = 10;
        public static final int buttonBack = 7;
        public static final int buttonStart = 8;
    
        //joysticks -->
        public static final int leftXAxis = 0;
        public static final int leftYAxis = 1;
        public static final int rightXAxis = 4; 
        public static final int rightYAxis = 3;
      }

    public final class k_chassis {
        public static final double kPTurning = 0.7;
        public static final double kPDrive = 0.3;
        public static final double kI = 0.0;
        public static final double kD = 0.0;
        public static final double kDeadband = 0.6;
        public static final double AccelerationUnitsPerSecond = 1.1;
        public static final double AngularAccelerationUnitsPerSecond = 1;
        public static final double MaxMetersPerSecond = 0.66;

        public static final int kCurrentLimit = 60;
        public static final double driveToDistance = 10.0;
        public static final double targetReached = 0.20;
        public static final double activateDrivePid = 0.60; 
        public static final double activatePidSpeedScaler = MaxMetersPerSecond/activateDrivePid;
    
        //dunno
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
    
    
        public static final double kTrackWidth = Units.inchesToMeters(22.75);
        public static final double kWheelBase = Units.inchesToMeters(22.75);
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
              new Translation2d(kWheelBase / 2, -kTrackWidth / 2), 
              new Translation2d(kWheelBase / 2, kTrackWidth / 2), 
              new Translation2d(-kWheelBase / 2, -kTrackWidth / 2), 
              new Translation2d(-kWheelBase / 2, kTrackWidth / 2)); 
              
        //Chassis Motor ports
        /* x1 on can, drive motors*/ 
        public final static int leftFrontMotorPort  = 21; 
        public final static int rightFrontMotorPort = 11;
        public final static int rightRearMotorPort  = 41; 
        public final static int leftRearMotorPort   = 31; 
    
        /* x2 on can, steering motors */
        public final static int leftFrontMotorSteerPort  = 22;
        public final static int rightFrontMotorSteerPort = 12;
        public final static int rightRearMotorSteerPort  = 42;
        public final static int leftRearMotorSteerPort   = 32;
        
      
        public final static double gyro = 0;

       
      }


      public static class autoConstants {
  
     
        //every auto distance constant in inches here unless specified otherwise
        //every auto rotation constant in degrees here unless specified otherwise
        //negative for counterclockwise or to left, positive for clockwise or to right
  
        /**
         * Below are drive constants for a hard-coded autonomous 
         * TODO: from side start calculations especially are roughly calculated 
         */
  
  
        // // I don't know what these are for???? keeping them in too see though
        // public static final double kMaxAngularSpeedRadiansPerSecond = //
        // k_chassis.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
        // public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
  
        // public static final double kPXController = 1.5;
        // public static final double kPYController = 1.5;
        // public static final double kPThetaController = 3;
  
        // public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
        //           new TrapezoidProfile.Constraints(
        //                   kMaxAngularSpeedRadiansPerSecond,
        //                   kMaxAngularAccelerationRadiansPerSecondSquared);
  
  
        public static final double kAutoDriveDistanceInchesOut = 60;
        public static final double kAutoDriveSpeed = 0.6;
        // public static final double kAutoRotationSp = 0.0;
        public static final double kAutoDriveDistanceToCoralFromMidStart = 88;
        public static final double kAutoDriveDistanceToCoralFromSideStart = 111;
        public static final double kAutoDriveDistanceMidFromLeft = 36;
        public static final double kAutoDriveDistanceMidFromRight = -36;
        public static final double kAutoTurnToCoralFromLeft = 8.51578;
        public static final double kAutoTurnToCoralFromRight = -8.51578;
  
        /**
         * Below are mechanism constants for hard-coded seesaw autonomous 
         * TODO: seesaw pickup and drop angles need to be confirmed through testing
         */
        public static final double seesawPickupAngle = -35; 
        public static final double seesawDropAngle = 35; 
        public static final double seesawVerticalAngle = -90; 
        public static final double seesawFlatAngle = 0; 
  
        
   
        /**
         * Below are mechanism constants for hard-coded jetpack + elevator autonomous
         * TODO: add constants here for jetpack and elevator hard-coded autonomous 
         */
        
  
      }


}
