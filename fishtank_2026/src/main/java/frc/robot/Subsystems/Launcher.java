package frc.robot.Subsystems;
import frc.robot.Subsystems.Motor;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Launcher extends Motor {



    public Launcher(String name, int canId, double kpPID, double kiPID, double kdPID)
    {
        super(name, canId, kpPID, kiPID, kdPID, false);


    }

    public void launchFuel(Pose3d targetPose)
    {
        double grav = -9.82;
        setTargetVelocity(0.25, false);
    }
}
