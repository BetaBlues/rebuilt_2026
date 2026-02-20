package frc.robot.Subsystems.Cameras;

import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Subsystems.ChassisGyro;

public class LeftCamera {
    public static PhotonCamera LeftCamera;

    public static AprilTagFieldLayout kTagLayout;

     public static Transform3d kRobotToCam;
     private PhotonTrackedTarget m_target;
     private Pose3d m_robotPose;
     private int m_apriltagId = -1;
     private boolean isIndex = false;
     private double[] xValues = new double[5];
     private double[] yValues = new double[5];
    private int count = 0;

     public LeftCamera() {
          //LeftCamera = new PhotonCamera("LeftCamera");
          LeftCamera = new PhotonCamera("LeftCamera");
          // RightCamera = new PhotonCamera("RightCamera");

          kTagLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);
          kRobotToCam = new Transform3d(new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0));
     }

     public Pose3d estimateLeftPose(Field2d field, ChassisGyro gyro) {
          List<PhotonPipelineResult> results = LeftCamera.getAllUnreadResults();
          
          if(results.size() < 1) {
               return null;
          }
          SmartDashboard.putNumber("Results size", results.size());
          try {
               PhotonPipelineResult result = results.get(results.size()-1);
               if (result.hasTargets()) {
                    isIndex = true;
                    m_target = result.getBestTarget();
                    m_apriltagId = m_target.getFiducialId();
                    if (kTagLayout.getTagPose(m_target.getFiducialId()).isPresent()) {
                         m_robotPose = PhotonUtils.estimateFieldToRobotAprilTag(m_target.getBestCameraToTarget(), kTagLayout.getTagPose(m_target.getFiducialId()).get(), kRobotToCam);
                         field.setRobotPose(m_robotPose.getX(), m_robotPose.getY(), gyro.getRotation2d());
                        xValues[count] = m_robotPose.getX();
                        yValues[count] = m_robotPose.getY();
                        count++;
                        if (count > 4) {
                            count = 0;
                        }
                        if 
                        field.setRobotPose(getXAverage(), getYAverage(), gyro.getRotation2d());
                        return m_robotPose;
                         //field.setRobotPose(5.0, 5.0, gyro.getRotation2d());
                    }
               }
            //    SmartDashboard.putNumber("Distance x", m_robotPose.getX());
            //    SmartDashboard.putNumber("Distance y", m_robotPose.getY());
            //    SmartDashboard.putNumber("Distance z", m_robotPose.getZ());
            //    SmartDashboard.putNumber("Distance to tag", m_target.getBestCameraToTarget().getX());
            //    SmartDashboard.putNumber("Angle to tag", m_target.getBestCameraToTarget().getY());
            //    SmartDashboard.putNumber("Delta to tag", m_target.getBestCameraToTarget().getZ());

               
          }

          catch(Exception e) {
               SmartDashboard.putBoolean("Has april tag target:", isIndex);
          }

          finally {

          }

        return null;
     }

     public double getXAverage() {
        double sum = 0;
        for (double x : xValues) {
            sum += x;
        }
        return sum / xValues.length;
     }

     public double getYAverage() {
        double sum = 0;
        for (double y : yValues) {
            sum += y;
        }
        return sum / yValues.length;
     }

     public void showData() {
          //SmartDashboard.putNumber("Left Apriltag Id", m_target.getFiducialId());
          SmartDashboard.putString("Left Camera Name", LeftCamera.getName());
          SmartDashboard.putBoolean("Left Camera Connected", LeftCamera.isConnected());
          SmartDashboard.putNumber("Left Camera Apriltag Id", m_apriltagId);
          // SmartDashboard.put
     } 
}
