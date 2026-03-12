package frc.robot.Subsystems;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;

import org.photonvision.targeting.PhotonPipelineResult;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;



public class Vision {

    private Camera m_leftCamera;
    private Camera m_middleCamera;
    private Camera m_rightCamera;

    private Pose3d m_leftPose;
    private Pose3d m_middlePose;
    private Pose3d m_rightPose;
    private Pose3d m_calcPose;

    private double x;
    private double y;
    private double z;
    private Rotation3d rot;
  
    public Vision() {
        // need to measure offsets
        m_leftCamera = new Camera("LeftCamera", -30*Math.PI/180, 0, 0.33, Math.toRadians(15));
        m_middleCamera = new Camera("MiddlCamera", 0, 0, 0.3937, Math.toRadians(15));
        m_rightCamera = new Camera("RightCamera", 30*Math.PI/180, 0, 0.33, Math.toRadians(15));
    }

    public Pose3d estimatePoseAllCam() {
        if (Constants.hasVision) {
            m_leftPose = m_leftCamera.estimateAveragePose();
            m_middlePose = m_middleCamera.estimateAveragePose();
            m_rightPose = m_rightCamera.estimateAveragePose();

            if (m_leftPose != null && m_middlePose != null && m_rightPose != null) {
                x += m_leftPose.getX() + m_middlePose.getX() + m_rightPose.getX();
                y += m_leftPose.getY() + m_middlePose.getY() + m_rightPose.getY();
                z += m_leftPose.getZ() + m_middlePose.getZ() + m_rightPose.getZ();
                rot = rot.plus(m_leftPose.getRotation()).plus(m_middlePose.getRotation()).plus(m_rightPose.getRotation()); 
                x /= 3;
                y /= 3;
                rot = rot.div(3);
                m_calcPose = new Pose3d(x, y, z, rot);
                return m_calcPose;
            }
            else {
                return null;
            }
        }
        else {
            return null;
        }
    }

    public void showAllData() {
        m_leftCamera.showData();
        m_middleCamera.showData();
        m_rightCamera.showData();
    }


}
