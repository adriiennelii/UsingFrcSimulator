package frc.robot.sensors;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.SimulationState;

public class MagicPositionSensor {
    private final SimulationState simulationState;
    
    public MagicPositionSensor(SimulationState simulationState) {
        this.simulationState = simulationState;
    }

    public Pose2d getRobotPosition() {
        return simulationState.getRobotPosition();
    }

    public Pose2d getRobotVelocity() {
        return simulationState.getRobotVelocity();
    }



}