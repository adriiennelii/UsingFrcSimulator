package frc.robot.subsystems;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.SimulationState;

public class SimulatorDriveSubsystemTest {

    private static final double epsilon = 1e-12;
    private SimulationState simulationState;

    @BeforeEach
    public void setup() {
        simulationState = new SimulationState();
    }

    @Test
    public void testUpdatePositionStationary() {
        Pose2d currentPosition = new Pose2d();
        Pose2d nextPosition = simulationState.calculateNextPosition(currentPosition, 0.0, 0.0, 1.0);
        assertEquals(currentPosition, nextPosition);   
    }

    @Test
    public void testUpdatePositionAlongXAxis() {
        Pose2d currentPosition = new Pose2d();
        Pose2d nextPosition = simulationState.calculateNextPosition(currentPosition, 2.0, 0.0, 3.0);
        assertEquals(6.0, nextPosition.getTranslation().getX(), epsilon);
        assertEquals(0.0, nextPosition.getTranslation().getY(), epsilon);
        assertEquals(0.0, nextPosition.getRotation().getRadians(), epsilon);
    }

    @Test
    public void testUpdatePositionAlongYAxis() {
        Pose2d currentPosition = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Math.PI / 2.0));
        Pose2d nextPosition = simulationState.calculateNextPosition(currentPosition, 2.0, 0.0, 3.0);
        assertEquals(0.0, nextPosition.getTranslation().getX(), epsilon);
        assertEquals(6.0, nextPosition.getTranslation().getY(), epsilon);
        assertEquals(90.0, nextPosition.getRotation().getDegrees(), epsilon);
    }

    @Test
    public void testUpdatePositionBackwards() {
        Pose2d currentPosition = new Pose2d(new Translation2d(0.0, 0.0), new Rotation2d(Math.PI / 2.0));
        Pose2d nextPosition = simulationState.calculateNextPosition(currentPosition, -2.0, 0.0, 3.0);
        assertEquals(0.0, nextPosition.getTranslation().getX(), epsilon);
        assertEquals(-6.0, nextPosition.getTranslation().getY(), epsilon);
        assertEquals(90.0, nextPosition.getRotation().getDegrees(), epsilon);
    }

    @Test
    public void testRotateLeft() {
        Pose2d currentPosition = new Pose2d();
        Pose2d nextPosition = simulationState.calculateNextPosition(currentPosition, 0.0, Math.PI / 4.0, 2.0);
        assertEquals(0.0, nextPosition.getTranslation().getX(), epsilon);
        assertEquals(0.0, nextPosition.getTranslation().getY(), epsilon);
        assertEquals(Math.PI / 2.0, nextPosition.getRotation().getRadians());
    }

    @Test
    public void testQuarterTurnRight() {
        Pose2d position = new Pose2d();
        for (int i=0; i<1000; ++i) {
            position = simulationState.calculateNextPosition(position, 1.0, -Math.PI / 2.0, 0.001);
        }
        assertEquals(-90.0, position.getRotation().getDegrees(), epsilon);
        // Not sure what the actual number here should be, just where it should be on the circle.
        assertEquals(position.getTranslation().getY(), -position.getTranslation().getX(), 0.001);
    }

    @Test
    public void testFullTurn() {
        Pose2d position = new Pose2d();
        // Travel PI meters per second, for 1 second, turning left at a rate of 2*PI radians/second
        for (int i=0; i<1000; ++i) {
            position = simulationState.calculateNextPosition(position, Math.PI, 2.0 * Math.PI, 0.001);
        }
        assertEquals(0.0, position.getRotation().getRadians(), 0.001);
        assertEquals(0.0, position.getTranslation().getX(), 0.001);
        assertEquals(0.0, position.getTranslation().getY(), 0.001);
    }

    @Test
    public void testLinearAcceleration() {
        SimulationState.SpeedPair speedPair = simulationState.calculateNextVelocity(0.0, 0.0, 2.0, 3.0, 0.0);
        assertEquals(5.8, speedPair.linear); // This is the velocity, plus the acceleration, minus the friction
        assertEquals(0.0, speedPair.rotational);
    }

}
