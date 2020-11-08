package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import frc.robot.util.Field2d;

public class SimulationState {
    private static final Field2d field2d = new Field2d();
    private double robotLinearSpeed;
    private double robotRotationalSpeed;

    public SimulationState() {
    }

    public void reset() {
        field2d.setRobotPose(new Pose2d());
        robotLinearSpeed = 0.0;
        robotRotationalSpeed = 0.0;
    }

    public Pose2d getRobotPosition() {
        return field2d.getRobotPose();
    }

    public void setPosition(Pose2d position) {
        field2d.setRobotPose(position);
    }

    public double getRobotLinearSpeed() {
        return robotLinearSpeed;
    }

    public void setRobotLinearSpeed(double robotLinearSpeed) {
        this.robotLinearSpeed = robotLinearSpeed;
    }

    public double getRobotRotationalSpeed() {
        return robotRotationalSpeed;
    }

    public void setRobotRotationalSpeed(double robotRotationalSpeed) {
        this.robotRotationalSpeed = robotRotationalSpeed;
    }

}
