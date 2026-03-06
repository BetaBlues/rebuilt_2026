// package frc.robot.Subsystems;
// import java.util.List;

// import org.photonvision.PhotonCamera;
// import org.photonvision.PhotonUtils;
// import org.photonvision.targeting.PhotonTrackedTarget;

// import edu.wpi.first.apriltag.AprilTagFieldLayout;
// import edu.wpi.first.apriltag.AprilTagFields;
// import edu.wpi.first.math.geometry.Pose3d;
// import edu.wpi.first.math.geometry.Transform3d;

// import org.photonvision.targeting.PhotonPipelineResult;
// import edu.wpi.first.math.geometry.Translation3d;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.math.geometry.Rotation3d;
// import edu.wpi.first.wpilibj.smartdashboard.Field2d;



// public class Vision {
//      public static PhotonCamera LeftCamera;
//      public static PhotonCamera MiddleCamera;
//      public static PhotonCamera RightCamera;

//      public static AprilTagFieldLayout kTagLayout;

//      public static Transform3d kRobotToCam;
//      private PhotonTrackedTarget m_target;
//      private Pose3d m_robotPose;
//      private int m_apriltagId = -1;
//      private boolean isIndex = false;

//      public Vision() {
        
//           MiddleCamera = new PhotonCamera("MiddleCamera");

//           kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
//           kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
//      }

//      public void estimateMiddlePose(Field2d field, ChassisGyro gyro) {
//           List<PhotonPipelineResult> results = MiddleCamera.getAllUnreadResults();
          
//           if(results.size() < 1) {
//                return;
//           }
//           SmartDashboard.putNumber("Results size", results.size());
//           try {
//                isIndex = false;
//                PhotonPipelineResult result = results.get(results.size()-1);
//                if (result.hasTargets()) {
//                     isIndex = true;
//                     m_target = result.getBestTarget();
//                     m_apriltagId = m_target.getFiducialId();
//                     if (kTagLayout.getTagPose(m_target.getFiducialId()).isPresent()) {
//                          m_robotPose = PhotonUtils.estimateFieldToRobotAprilTag(m_target.getBestCameraToTarget(), kTagLayout.getTagPose(m_target.getFiducialId()).get(), kRobotToCam);
//                          //field.setRobotPose(m_robotPose.getX(), m_robotPose.getY(), m_robotPose.getRotation().toRotation2d()); //gyro.getRotation2d()
//                          //field.setRobotPose(5.0, 5.0, gyro.getRotation2d());
//                     }
//                }
             
//                SmartDashboard.putNumber("Distance x", m_robotPose.getX());
//                SmartDashboard.putNumber("Distance y", m_robotPose.getY());
//                SmartDashboard.putNumber("Distance z", m_robotPose.getZ());
              
//                SmartDashboard.putNumber("Distance to tag", m_target.getBestCameraToTarget().getX());
//                SmartDashboard.putNumber("Angle to tag", m_target.getBestCameraToTarget().getY());
//                SmartDashboard.putNumber("Delta to tag", m_target.getBestCameraToTarget().getZ());
//           }

//           catch(Exception e) {
//                SmartDashboard.putBoolean("Has april tag target:", isIndex);
//           }

//           finally {

//           }


//      }

//      public void showData() {
//           //SmartDashboard.putNumber("Left Apriltag Id", m_target.getFiducialId());
//           SmartDashboard.putString("Middle Camera Name", MiddleCamera.getName());
//           SmartDashboard.putBoolean("Camera Connected", MiddleCamera.isConnected());
//           SmartDashboard.putNumber("Apriltag Id", m_apriltagId);
//           // SmartDashboard.put
//      } 

// }
