package frc.robot.Subsystems;

import java.util.List;
import java.util.ArrayList;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.photonvision.PhotonPoseEstimator;

public class Camera {
     public PhotonCamera camera;

     public static AprilTagFieldLayout kTagLayout;

     public PhotonPoseEstimator m_estimator;

     public Transform3d kRobotToCam;
     private PhotonTrackedTarget m_target;
     private Pose3d m_robotPose;
     private Pose3d m_Pose3d;
     private int m_apriltagId = -1;
     private boolean isIndex = false;
     private double[] xValues = new double[5];
     private double[] yValues = new double[5];
     private double[] zValues = new double[5];

     private List<PhotonTrackedTarget> targets;

     public Camera(String cameraName, double yaw, double x, double y, double pitch) {
          camera = new PhotonCamera(cameraName);
          if (null == camera || !camera.isConnected()) {
               System.out.println(cameraName + " not found");
               throw new NullPointerException(cameraName + " not found");
          }
          System.out.println("Setting up " + cameraName);
          kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
          kRobotToCam = new Transform3d(new Translation3d(x, y, 0.435), new Rotation3d(0, pitch, yaw));

          camera.setDriverMode(false);
     }

     public boolean isValid() {
          return (null != camera && camera.isConnected());
     }

     public String getName() {
          return camera.getName();
     }
     public Pose3d estimateAveragePose() {
          if (!isValid()) return null; // Did is disconnect....

          List<PhotonPipelineResult> results = camera.getAllUnreadResults();
          ArrayList<Pose3d> estimatedPoses = new ArrayList<Pose3d>();
          
          SmartDashboard.putNumber("Results size", results.size());

          if(results.size() < 1) {
               isIndex = false;
               return null;

          }
          try {
               PhotonPipelineResult result = results.get(results.size()-1);
               if (result.hasTargets()) {
                    SmartDashboard.putNumber("Targets size", result.getTargets().size());
                    isIndex = true;
                    targets = result.getTargets();
                    for (int i = 0; i < targets.size(); i++) {
                         if (targets.get(i).getPoseAmbiguity() < 0.2) {
                              m_target = targets.get(i);
                              if (kTagLayout.getTagPose(m_target.getFiducialId()).isPresent()) {
                              estimatedPoses.add(PhotonUtils.estimateFieldToRobotAprilTag(m_target.getBestCameraToTarget(), kTagLayout.getTagPose(m_target.getFiducialId()).get(), kRobotToCam));
                              }
                         }
                    }
                    double x = 0.0;
                    double y = 0.0;
                    double z = 0.0;
                    Rotation3d rot = new Rotation3d();
                    for (int i = 0; i < estimatedPoses.size(); i++) {
                         x += estimatedPoses.get(i).getX();
                         y += estimatedPoses.get(i).getY();
                         z += estimatedPoses.get(i).getZ();
                         rot = rot.plus(estimatedPoses.get(i).getRotation());
                    }
                    x /= estimatedPoses.size();
                    y /= estimatedPoses.size();
                    z /= estimatedPoses.size();
                    rot = rot.div(estimatedPoses.size());
                    m_robotPose = new Pose3d(x, y, z, rot);

                    m_apriltagId = m_target.getFiducialId();
                        
                        SmartDashboard.putNumber("Distance x", m_robotPose.getX());
                         SmartDashboard.putNumber("Distance y", m_robotPose.getY());
                         SmartDashboard.putNumber("Distance z", m_robotPose.getZ());
                         SmartDashboard.putNumber("Distance to tag", m_target.getBestCameraToTarget().getX());
                         SmartDashboard.putNumber("Angle to tag", m_target.getBestCameraToTarget().getY());
                         SmartDashboard.putNumber("Delta to tag", m_target.getBestCameraToTarget().getZ());
                        return m_robotPose;
               }


               
          }

          catch(Exception e) {
               SmartDashboard.putBoolean("Has april tag target:", isIndex);
          }

          finally {

          }

        return null;
     }
     

     public Pose3d estimatePose(Field2d field, ChassisGyro gyro) {
          if (!isValid()) return null; // Did is disconnect....

          List<PhotonPipelineResult> results = camera.getAllUnreadResults();
          
          SmartDashboard.putNumber("Results size", results.size());

          if(results.size() < 1) {
               isIndex = false;
               return null;

          }
          try {
               PhotonPipelineResult result = results.get(results.size()-1);
               if (result.hasTargets()) {
                    SmartDashboard.putNumber("Targets size", result.getTargets().size());
                    isIndex = true;
                    m_target = result.getBestTarget();
                    m_apriltagId = m_target.getFiducialId();
                    if (kTagLayout.getTagPose(m_target.getFiducialId()).isPresent()) {
                         m_robotPose = PhotonUtils.estimateFieldToRobotAprilTag(m_target.getBestCameraToTarget(), kTagLayout.getTagPose(m_target.getFiducialId()).get(), kRobotToCam);
                        SmartDashboard.putNumber("Distance x", m_robotPose.getX());
                         SmartDashboard.putNumber("Distance y", m_robotPose.getY());
                         //SmartDashboard.putNumber("Distance z", m_robotPose.getZ());
                         SmartDashboard.putNumber("Distance to tag", m_target.getBestCameraToTarget().getX());
                         // SmartDashboard.putNumber("Angle to tag", m_target.getBestCameraToTarget().getY());
                         // SmartDashboard.putNumber("Delta to tag", m_target.getBestCameraToTarget().getZ());
                        return m_robotPose;
                    }
               }


               
          }

          catch(Exception e) {
               SmartDashboard.putBoolean("Has april tag target:", isIndex);
          }

          finally {

          }

        return null;
     }

     public Pose3d getPose3d() {
          return m_robotPose;
     }

     public Pose2d getPose2d() {
          return m_robotPose.toPose2d();
     }

     // might need to change april tags b/c they're all of them right now, do we want just the front two
     public Transform3d getHubDistance() {
          if (!isValid()) return null; // Did is disconnect....

          if (targets != null) {
               if (DriverStation.getAlliance().get() == Alliance.Blue) {
                    for (int i = 0; i < targets.size(); i++) {
                         PhotonTrackedTarget t = targets.get(i);
                         int id = t.getFiducialId();
                         if (/*id == 18 || id == 19 || id == 20 || id == 21 || id == 24 ||*/ id == 25 || id == 26 /*|| id == 27*/) {
                              if (t.getPoseAmbiguity() < 0.2) {
                                   return t.getBestCameraToTarget();
                              }
                         }
                    }
               }
               else if (DriverStation.getAlliance().get() == Alliance.Red) {
                    for (int i = 0; i < targets.size(); i++) {
                         PhotonTrackedTarget t = targets.get(i);
                         int id = t.getFiducialId();
                         if (/*id == 2 || id == 3 || id == 4 || id == 5 || id == 8 ||*/ id == 9 || id == 10 /*|| id == 11*/) {
                              if (t.getPoseAmbiguity() < 0.2) {
                                   return t.getBestCameraToTarget();
                              }
                         }
                    }
               }
          }
          return null;
     }

     public void showData() {
          try {
            if (null != camera) {
              SmartDashboard.putString(camera.getName()  + "Camera Name", camera.getName());
              SmartDashboard.putBoolean(camera.getName() + " Camera Connected", camera.isConnected());
              SmartDashboard.putNumber(camera.getName()  + " Camera Apriltag Id", m_apriltagId);
            }
          }
          catch (Exception e) {
             System.out.println("Faild to show camera data: " + e);
          }
     }
}
