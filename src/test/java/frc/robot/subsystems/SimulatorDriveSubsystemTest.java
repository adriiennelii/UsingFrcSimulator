package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SimulatorDriveSubsystemTest {

    private static final double epsilon = 1e-12;

    @Test
    public void testUpdatePositionStationary() {
        Pose2d currentPosition = new Pose2d();
        Pose2d nextPosition = SimulatorDriveSubsystem.calculateNextPosition(currentPosition, 0.0, 0.0, 1.0);
        assertEquals(currentPosition, nextPosition);   
    }

    @Test
    public void testUpdatePositionAlongXAxis() {
        Pose2d currentPosition = new Pose2d();
        Pose2d nextPosition = SimulatorDriveSubsystem.calculateNextPosition(currentPosition, 2.0, 0.0, 3.0);
        assertEquals(6.0, nextPosition.getTranslation().getX(), epsilon);
        assertEquals(0.0, nextPosition.getTranslation().getY(), epsilon);
        assertEquals(0.0, nextPosition.getRotation().getRadians(), epsilon);
    }

    @Test
    public void testUpdatePositionAlongYAxis() {
        Pose2d currentPosition = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Math.PI / 2.0));
        Pose2d nextPosition = SimulatorDriveSubsystem.calculateNextPosition(currentPosition, 2.0, 0.0, 3.0);
        assertEquals(0.0, nextPosition.getTranslation().getX(), epsilon);
        assertEquals(6.0, nextPosition.getTranslation().getY(), epsilon);
        assertEquals(90.0, nextPosition.getRotation().getDegrees(), epsilon);
    }

    @Test
    public void testUpdatePositionBackwards() {
        Pose2d currentPosition = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Math.PI / 2.0));
        Pose2d nextPosition = SimulatorDriveSubsystem.calculateNextPosition(currentPosition, -2.0, 0.0, 3.0);
        assertEquals(0.0, nextPosition.getTranslation().getX(), epsilon);
        assertEquals(-6.0, nextPosition.getTranslation().getY(), epsilon);
        assertEquals(90.0, nextPosition.getRotation().getDegrees(), epsilon);
    }
}
