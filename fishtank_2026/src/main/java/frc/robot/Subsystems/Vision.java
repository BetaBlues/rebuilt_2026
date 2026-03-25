package frc.robot.Subsystems;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {

    private Camera m_leftCamera = null;
    private Camera m_middleCamera = null;
    private Camera m_rightCamera = null;
    List<Camera> activeCameras = new ArrayList<>();

    private Pose3d m_calcPose = null;

    private Transform3d m_hubDist = null;

    public Vision() {
        // need to measure offsets
        boolean showState = true;
        m_leftCamera = setupCamera("LeftCamera", -30*Math.PI/180, 0, 0.33, Math.toRadians(15), showState);
        m_middleCamera = setupCamera("MiddleCamera", 0, 0, 0.3937, Math.toRadians(15), showState);
        m_rightCamera = setupCamera("RightCamera", 30*Math.PI/180, 0, 0.33, Math.toRadians(15), showState);
        
        if (null != m_leftCamera) activeCameras.add(m_leftCamera);
        if (null != m_middleCamera) activeCameras.add(m_middleCamera);
        if (null != m_rightCamera) activeCameras.add(m_rightCamera);
    }

    public boolean hasVision() {
        for (Camera cam : activeCameras) {
            if (cam.isValid()) return true;
        }
        return false;
    }

    private Camera setupCamera(String cameraName, double yaw, double x, double y, double pitch, boolean showConnectionState) {
        Camera cam = null;
        try{
            cam = new Camera(cameraName, yaw, x, y, pitch);
        }
        catch (Exception e) {
            System.out.println("Failed to connect to " + cameraName + ": "  + e);
        }
        if (showConnectionState) SmartDashboard.putBoolean("Have " + cameraName, cam != null);
        return cam;
    }

    private Pose3d estimatePoseAllCam() {
        if (!hasVision())
            return null;

        try {
            List<Rotation3d> rot = new ArrayList<>();
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;

            for (Camera cam : activeCameras) {
                if (!cam.isValid())
                    continue;
                Pose3d pose = cam.estimateAveragePose();
                if (null != pose) {
                  x += pose.getX();
                  y += pose.getY();
                  z += pose.getZ();
                  rot.add(pose.getRotation());
                }
            }
            if (rot.size() > 0) {
                m_calcPose = new Pose3d(x / rot.size(), y / rot.size(), z / rot.size(),
                    averageRotation(rot));
            }
            else m_calcPose = null;

        } catch (Exception e) {
            m_calcPose = null;
        }
        return m_calcPose;
    }

    public Transform3d getHubDistance() {
        return m_hubDist;
    }

    public Pose3d getEstimatePose() {
        return m_calcPose;
    }

    // Just to compile
    private Rotation3d averageRotation(List<Rotation3d> rotations) {
        if (0 == rotations.size()) return new Rotation3d();
        if (1 == rotations.size()) return rotations.get(0);

        // Calculate from here out.
        return rotations.get(0);
    }

    private Transform3d calculateHubDistanceAll() {
        if (!hasVision())
            return null;

        try {
            List<Rotation3d> rot = new ArrayList<>();
            double x = 0.0;
            double y = 0.0;
            double z = 0.0;

            for (Camera cam : activeCameras) {
                if (!cam.isValid())
                    continue;
                Transform3d dist = cam.getHubDistance();
                if (null != dist) {
                  x += dist.getX();
                  y += dist.getY();
                  z += dist.getZ();
                  rot.add(dist.getRotation());
                }
            }

            if (rot.size() > 0) {
                m_hubDist = new Transform3d(x / rot.size(), y / rot.size(), z / rot.size(),
                    averageRotation(rot));
            }
            else m_calcPose = null;
        } catch (Exception e) {
            m_hubDist = null;
        }

        return m_hubDist;
    }

    @Override
    public void periodic() {
        estimatePoseAllCam();
        calculateHubDistanceAll();
        showAllData();
    }

    public void showAllData() {
        if (null != m_leftCamera) m_leftCamera.showData();
        if (null != m_middleCamera) m_middleCamera.showData();
        if (null != m_rightCamera) m_rightCamera.showData();
        
        SmartDashboard.putBoolean("Hub: ", m_hubDist != null);
        if (m_hubDist != null) {
            SmartDashboard.putString("Distance to hub: ", m_hubDist.toString());
        
            SmartDashboard.putNumber("Hub Distance X", m_hubDist.getX());
            SmartDashboard.putNumber("Hub Distance Y", m_hubDist.getY());
            SmartDashboard.putNumber("Hub Distance Rotation", Math.toDegrees(m_hubDist.getRotation().getAngle()));
            SmartDashboard.putBoolean("Can Launch", m_hubDist.getX() <= 3 && m_hubDist.getX() >= 2.8);
        } 

        if (m_calcPose != null) SmartDashboard.putString("Current Pose: ", m_calcPose.toString());
    }
}
