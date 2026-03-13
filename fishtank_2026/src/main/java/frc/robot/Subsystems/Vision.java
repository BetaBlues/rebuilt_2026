package frc.robot.Subsystems;
import edu.wpi.first.math.geometry.Pose3d;

import frc.robot.Constants;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;




public class Vision {

    private Camera m_leftCamera;
    private Camera m_middleCamera;
    private Camera m_rightCamera;

    private Pose3d m_leftPose;
    private Pose3d m_middlePose;
    private Pose3d m_rightPose;
    private Pose3d m_calcPose;

    private Transform3d m_leftHub;
    private Transform3d m_middleHub;
    private Transform3d m_rightHub;
    private Transform3d m_hubDist;

    private double x;
    private double y;
    private double z;
    private Rotation3d rot;

    private double xHub;
    private double yHub;
    private double zHub;
    private Rotation3d rotHub;

    private boolean canSeeHub;
  
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

    public Transform3d getHubDistanceAll() {

        m_leftHub = m_leftCamera.getHubDistance();
        m_middleHub = m_middleCamera.getHubDistance();
        m_rightHub = m_rightCamera.getHubDistance();

        if (m_leftHub != null && m_middleHub != null && m_rightHub != null) {
                xHub = m_leftHub.getX() + m_middleHub.getX() + m_rightHub.getX();
                yHub = m_leftHub.getY() + m_middleHub.getY() + m_rightHub.getY();
                zHub = m_leftHub.getZ() + m_middleHub.getZ() + m_rightHub.getZ();
                rotHub = rotHub.plus(m_leftHub.getRotation()).plus(m_middleHub.getRotation()).plus(m_rightHub.getRotation()); 
                xHub /= 3;
                yHub /= 3;
                rotHub = rotHub.div(3);
                m_hubDist = new Transform3d(xHub, yHub, zHub, rotHub);
                canSeeHub = true;
            }
            else {
                if(m_leftHub != null && m_middleHub != null) {
                    xHub = m_leftHub.getX() + m_middleHub.getX();
                    yHub = m_leftHub.getY() + m_middleHub.getY();
                    zHub = m_leftHub.getZ() + m_middleHub.getZ();
                    rotHub = rotHub.plus(m_leftHub.getRotation()).plus(m_middleHub.getRotation()); 
                    xHub /= 3;
                    yHub /= 3;
                    rotHub = rotHub.div(2);
                    m_hubDist = new Transform3d(xHub, yHub, zHub, rotHub);
                    canSeeHub = true;
                }
                else if(m_rightHub != null && m_middleHub != null) {
                    xHub = m_rightHub.getX() + m_middleHub.getX();
                    yHub = m_rightHub.getY() + m_middleHub.getY();
                    zHub = m_rightHub.getZ() + m_middleHub.getZ();
                    rotHub = rotHub.plus(m_rightHub.getRotation()).plus(m_middleHub.getRotation()); 
                    xHub /= 3;
                    yHub /= 3;
                    rotHub = rotHub.div(2);
                    m_hubDist = new Transform3d(xHub, yHub, zHub, rotHub);
                    canSeeHub = true;
                }
                else if (m_leftHub != null && m_middleHub == null && m_rightHub == null) {
                    xHub = m_leftHub.getX();
                    yHub = m_leftHub.getY();
                    zHub = m_leftHub.getZ();
                    rotHub = m_leftHub.getRotation();
                    m_hubDist = new Transform3d(xHub, yHub, zHub, rotHub);
                    canSeeHub = true;
                }
                else if (m_leftHub == null && m_middleHub != null && m_rightHub == null) {
                    xHub = m_middleHub.getX();
                    yHub = m_middleHub.getY();
                    zHub = m_middleHub.getZ();
                    rotHub = m_middleHub.getRotation();
                    m_hubDist = new Transform3d(xHub, yHub, zHub, rotHub);
                    canSeeHub = true;
                }
                else if (m_leftHub == null && m_middleHub == null && m_rightHub != null) {
                    xHub = m_rightHub.getX();
                    yHub = m_rightHub.getY();
                    zHub = m_rightHub.getZ();
                    rotHub = m_rightHub.getRotation();
                    m_hubDist = new Transform3d(xHub, yHub, zHub, rotHub);
                    canSeeHub = true;
                }
                else {
                    m_hubDist = null;
                }
                // canSeeHub = false;
                // return null;
            }
            
            return m_hubDist;
    }

    public void showAllData() {
        m_leftCamera.showData();
        m_middleCamera.showData();
        m_rightCamera.showData();
    }


}
