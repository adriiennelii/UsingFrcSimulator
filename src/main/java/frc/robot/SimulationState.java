package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.util.Field2d;

public class SimulationState {
    private final Field2d field2d = new Field2d();
    private Pose2d robotVelocity;

    public SimulationState() {
        robotVelocity = new Pose2d();
    }

    public Pose2d getRobotPosition() {
        return field2d.getRobotPose();
    }

    public Pose2d getRobotVelocity() {
        return robotVelocity;
    }

    public void setVelocity(Pose2d velocity) {
        this.robotVelocity = velocity;
    }

    public void setPosition(Pose2d position) {
        field2d.setRobotPose(position);
    }

}
