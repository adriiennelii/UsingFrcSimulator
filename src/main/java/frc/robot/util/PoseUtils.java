package frc.robot.util;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class PoseUtils {
    public static Rotation2d getHeading(Pose2d current, Pose2d target) {
        Translation2d difference = target.getTranslation().minus(current.getTranslation());
        // Get the angle in field coordinates that represents this difference
        Rotation2d fieldRotation = new Rotation2d(difference.getX(), difference.getY());

        // Now subtract the current heading, to get the turn needed to face the target
        return current.getRotation().minus(fieldRotation);
    }

    // All distances are measured in meters
    public static double getDistance(Pose2d current, Pose2d target) {
        Translation2d difference = target.getTranslation().minus(current.getTranslation());
        return difference.getNorm();
    }
}
