package frc.robot;


public class Constants {
    public final static boolean hasGyro = true;
    public final static boolean hasSwerve = true;
    public final static boolean hasMotor = false;
    public final static boolean hasLauncher = true;
    public final static boolean hasIntake = true;
    public final static boolean hasClimber = true;
    public final static boolean hasCanCoder = true;
    public final static boolean hasPWMEncoder = !hasCanCoder;
    public final static boolean hasUsbCameras = true;
    public final static boolean hasVision = true;
    public final static boolean hasPathPlanner = true;
    public static boolean testMotors = false;

    //REMEMBER TO CHANGE THIS WHEN CHANGING BETWEEN SWERVE, Cancoder is 0, pwm is 1
    public static int enc = (hasCanCoder ? 0 : 1);

   
    public static final int[] rightFrontAbsEncPort = {13, 1};
    public static final int[] leftFrontAbsEncPort = {23, 2};
    public static final int[] leftRearAbsEncPort = {33, 3};
    public static final int[] rightRearAbsEncPort = {43, 4};
    
    public static final double leftFrontAbsOffset =  0.238;
    public static final double rightFrontAbsOffset = 0.353;
    public static final double leftRearAbsOffset = 0.749;
    public static final double rightRearAbsOffset= 0.363;

    public static final double steerEncoderRatio = 3.406;
    public static final double driveEncoderRatio = 10.0;


    public static class SpinMotorConstants {
      //gear box is 1:10
      public static final int kCanId = 9;
      public static final int kCurrentLimit = 60;
      public static final double MaxRotationSpeed= 0.5; //change to a lower speed to make smoother //0.04
      public static final double launcherSpin = .75;
      public static final int absPort = 8; //5
      public static final double absoluteOffset = 0.0;
     
    

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
      public static final double rotsPerMeter = 3.058;
      public static final double feedforward = 0.0;

    }

    public static class LauncherConstants {
      public static final double pos_kP = 0.15;
      public static final double pos_kI = 0.0;
      public static final double pos_kD = 0.0;
      public static final double feedforward = 1/473.0;
    }

    public static class IntakeConstants {
      public static final double pos_kP = 0.4;
      public static final double pos_kI = 0.0;
      public static final double pos_kD = 0.0;
      public static final double feedforward = 0.0;
    }
    
    public static class ClimberConstants {
     
      public static final int absPort = 5; //5
      public static final double startPos = 0.0;
      public static final double ticsPerMeters= 1.0; //meters to rotation
      public static final double setpoint0 = 0.0;
      public static final double setpoint1 = 10.0;
      public static final double setpoint2 = -10.0;

      public static final double pos_kP = 0.4;
      public static final double pos_kI = 0.0;
      public static final double pos_kD = 0.0;

      public static final double Max_Height = 0.6;
      public static final double Min_Height = 0.4;
      public static final double feedforward = 0.0;
  
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
        public static final double MaxMetersPerSecond = 0.90;

        public static final int kCurrentLimit = 60;
        public static final double driveToDistance = 10.0;
        public static final double targetReached = 0.20;
        public static final double activateDrivePid = 0.60; 
        public static final double activatePidSpeedScaler = MaxMetersPerSecond/activateDrivePid;
    
        //dunno
        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;
        public static final double kDriveEncoderDistancePerRotation = 0.0435;
    

              
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


      
      public static class VisionConstants {
        
      }


}
