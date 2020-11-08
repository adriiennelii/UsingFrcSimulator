package frc.robot.sensors;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.SimulationState;

public class MagicTargetSensor {
    private final SimulationState simulationState;
    public MagicTargetSensor(SimulationState simulationState) {
        this.simulationState = simulationState;
    }

    public double getHeadingToTarget() {
        Translation2d target = simulationState.getTargetPosition();
        Pose2d robotPosition = simulationState.getRobotPosition();
        Translation2d robotTranslation = robotPosition.getTranslation();
        Translation2d vectorToTarget = robotTranslation.minus(target);
        Rotation2d rotation = new Rotation2d(vectorToTarget.getX(), vectorToTarget.getY());
        Rotation2d robotRotation = robotPosition.getRotation();
        return rotation.minus(robotRotation).getDegrees();
    }

    public double getDistanceToTarget() {
        Translation2d target = simulationState.getTargetPosition();
        Pose2d robotPosition = simulationState.getRobotPosition();
        Translation2d robotTranslation = robotPosition.getTranslation();
        return robotTranslation.getDistance(target);
    }
}
