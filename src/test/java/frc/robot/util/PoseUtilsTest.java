package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class PoseUtilsTest {
    private static final double epsilon = 0.000000001;
    @Test
    public void testZeroDistance() {
        Pose2d current = new Pose2d(new Translation2d(1.2, 3.4), new Rotation2d(0.1));
        Pose2d target = new Pose2d(new Translation2d(1.2, 3.4), new Rotation2d(0.0));
        double distance = PoseUtils.getDistance(current, target);
        assertEquals(0.0, distance);
    }
    
    @Test
    public void testPythagoreanTheorem() {
        Pose2d current = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(1.2));
        Pose2d target = new Pose2d(new Translation2d(3.0, 4.0), new Rotation2d(2.3));

        double distance = PoseUtils.getDistance(current, target);
        assertEquals(5.0, distance);
    }

    @Test
    public void testDirectionForward() {
        Pose2d current = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
        Pose2d target = new Pose2d(new Translation2d(1.0, 0.0), new Rotation2d(0.0));

        Rotation2d heading = PoseUtils.getHeading(current, target);
        assertEquals(0.0, heading.getRadians());
    }

    @Test
    public void testDirectionBackward() {
        Pose2d current = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
        Pose2d target = new Pose2d(new Translation2d(-1.0, 0.0), new Rotation2d(0.0));

        Rotation2d heading = PoseUtils.getHeading(current, target);
        assertEquals(Math.PI, Math.abs(heading.getRadians()), epsilon);
    }

    @Test
    public void testTurnLeft45Degrees() {
        Pose2d current = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
        Pose2d target = new Pose2d(new Translation2d(1.0, 1.0), new Rotation2d(0.0));

        Rotation2d heading = PoseUtils.getHeading(current, target);
        assertEquals(-45.0,heading.getDegrees(), epsilon);

    }

    @Test
    public void testTurnRight45Degrees() {
        Pose2d current = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(0.0));
        Pose2d target = new Pose2d(new Translation2d(1.0, -1.0), new Rotation2d(0.0));

        Rotation2d heading = PoseUtils.getHeading(current, target);
        assertEquals(45.0, heading.getDegrees(), epsilon);

    }

}