package frc.robot.sensors;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.SimulationState;

public class MagicTargetSensorTest {
    SimulationState simulationState;
    MagicTargetSensor sensor;

    @BeforeEach
    public void setup() {
        simulationState = new SimulationState();
        sensor = new MagicTargetSensor(simulationState);
    }
    
    @Test
    public void testRightOnTarget() {
        simulationState.setRobotPosition(new Pose2d(new Translation2d(4.0, 4.0), new Rotation2d(2.0)));
        assertEquals(0.0, sensor.getDistanceToTarget());
    }

    @Test
    public void testItsRightInFrontOfYou() {
        simulationState.setRobotPosition(new Pose2d(new Translation2d(0.0, 4.0), new Rotation2d(0.0)));
        assertEquals(4.0, sensor.getDistanceToTarget());
        assertEquals(0.0, sensor.getHeadingToTarget());
    }

    @Test
    public void testToTheLeftToTheLeftToTheLeft() {
        simulationState.setRobotPosition(new Pose2d());
        assertEquals(45.0, sensor.getHeadingToTarget());
        assertEquals(Math.sqrt(4.0*4.0 + 4.0*4.0), sensor.getDistanceToTarget()); // Pythagorean theorem!
    }

    @Test
    public void testToTheRightToTheRightToTheRight() {
        simulationState.setRobotPosition(new Pose2d(new Translation2d(0.0, 8.0), new Rotation2d(0.0)));
        assertEquals(-45.0, sensor.getHeadingToTarget());
        assertEquals(Math.sqrt(4.0*4.0 + 4.0*4.0), sensor.getDistanceToTarget()); // Pythagorean theorem!
    }

}