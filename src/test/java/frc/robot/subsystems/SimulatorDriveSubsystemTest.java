package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;

public class SimulatorDriveSubsystemTest {
    @Test
    public void testUpdatePositionStationary() {
        Pose2d currentPosition = new Pose2d();
        Pose2d currentVelocity = new Pose2d();
        Pose2d nextPosition = SimulatorDriveSubsystem.calculateNextPosition(currentPosition, currentVelocity, 1.0);
        assertEquals(currentPosition, nextPosition);   
    }

    @Test
    public void testUpdatePositionAlongXAxis() {
        Pose2d currentPosition = new Pose2d();
        Pose2d currentVelocity = new Pose2d(new Translation2d(2.0, 0.0), new Rotation2d());
        Pose2d nextPosition = SimulatorDriveSubsystem.calculateNextPosition(currentPosition, currentVelocity, 3.0);
        assertEquals(6.0, nextPosition.getTranslation().getX());
        assertEquals(0.0, nextPosition.getTranslation().getY());
        assertEquals(0.0, nextPosition.getRotation().getRadians());
    }

    @Test
    public void testUpdatePositionAlongYAxis() {
        Pose2d currentPosition = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(90.0));
        Pose2d currentVelocity = new Pose2d(new Translation2d(0.0, 2.0), new Rotation2d());
        Pose2d nextPosition = SimulatorDriveSubsystem.calculateNextPosition(currentPosition, currentVelocity, 3.0);
        assertEquals(0.0, nextPosition.getTranslation().getX());
        assertEquals(6.0, nextPosition.getTranslation().getY());
        assertEquals(90.0, nextPosition.getRotation().getDegrees());
    }

}