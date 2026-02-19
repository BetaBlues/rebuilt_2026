package frc.robot.Subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class PoseEstimation {

    // /**
    //  * Fuses three pose estimates into one using weighted averaging.
    //  * @param p1 First pose
    //  * @param w1 Weight/confidence for first pose (higher = more trust)
    //  * @param p2 Second pose
    //  * @param w2 Weight/confidence for second pose
    //  * @param p3 Third pose
    //  * @param w3 Weight/confidence for third pose
    //  * @return Fused Pose2d
    //  */
    // public static Pose2d fusePoses(Pose2d p1, double w1, Pose2d p2, double w2, Pose2d p3, double w3) {
    //     double totalWeight = w1 + w2 + w3;
    //     if (totalWeight <= 0) {
    //         throw new IllegalArgumentException("Total weight must be > 0");
    //     }

    //     // Weighted average for X and Y
    //     double fusedX = (p1.getX() * w1 + p2.getX() * w2 + p3.getX() * w3) / totalWeight;
    //     double fusedY = (p1.getY() * w1 + p2.getY() * w2 + p3.getY() * w3) / totalWeight;

    //     // Handle rotation averaging properly (avoid wrap-around issues)
    //     double sinSum = Math.sin(p1.getRotation().getRadians()) * w1
    //                   + Math.sin(p2.getRotation().getRadians()) * w2
    //                   + Math.sin(p3.getRotation().getRadians()) * w3;
    //     double cosSum = Math.cos(p1.getRotation().getRadians()) * w1
    //                   + Math.cos(p2.getRotation().getRadians()) * w2
    //                   + Math.cos(p3.getRotation().getRadians()) * w3;
    //     double fusedTheta = Math.atan2(sinSum, cosSum);

    //     return new Pose2d(fusedX, fusedY, new Rotation2d(fusedTheta));
    // }

    // // // Example usage for robot periodic
    
    // //     Pose2d cam1 = new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(10));
    // //     Pose2d cam2 = new Pose2d(2.1, 2.9, Rotation2d.fromDegrees(12));
    // //     Pose2d cam3 = new Pose2d(1.9, 3.1, Rotation2d.fromDegrees(9));

    // //     // Example: trust cam1 and cam3 most
    // //     Pose2d fused = fusePoses(cam1, 0.5, cam2, 0.3, cam3, 0.5);

    // //     System.out.println("Fused Pose: " + fused);
    
}
