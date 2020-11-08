package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.subsystems.SimulatorDriveSubsystem;
import frc.robot.subsystems.SimulatorDriveSubsystem.SpeedPair;
import frc.robot.util.Field2d;

public class SimulationState {
    private static final Field2d field2d = new Field2d();
    private double robotLinearSpeed;
    private double robotRotationalSpeed;
    public long lastNanos;


    public SimulationState() {
        lastNanos = System.nanoTime();
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

	public static Pose2d calculateNextPosition(Pose2d currentPosition, double linearSpeed, double rotationalSpeed, double intervalSeconds) {
	    Translation2d currentVelocity = new Translation2d(linearSpeed, 0.0).rotateBy(currentPosition.getRotation());
	    Translation2d nextTranslation = currentPosition.getTranslation().plus(currentVelocity.times(intervalSeconds));
	    Rotation2d nextRotation = currentPosition.getRotation().plus(new Rotation2d(rotationalSpeed * intervalSeconds));
	    return new Pose2d(nextTranslation, nextRotation);
	  }

	public static SimulatorDriveSubsystem.SpeedPair calculateNextVelocity(double currentLinearSpeed, double currentRotationalSpeed, double intervalSeconds, double linearAcceleration, double rotationalAcceleration) {
	    double nextLinearSpeed = currentLinearSpeed + linearAcceleration * intervalSeconds;
	    double linearFriction = SimulatorDriveSubsystem.LINEAR_FRICTION_COEFFICIENT * intervalSeconds * Math.signum(nextLinearSpeed);
	    if (Math.abs(nextLinearSpeed) < Math.abs(linearFriction)) {
	      nextLinearSpeed = 0.0;
	    } else {
	      nextLinearSpeed -= linearFriction;
	    }
	    double nextRotationalSpeed = currentRotationalSpeed + rotationalAcceleration * intervalSeconds;
	    double rotationalFriction = SimulatorDriveSubsystem.ROTATIONAL_FRICTION_COEFFICIENT * intervalSeconds * Math.signum(nextRotationalSpeed);
	    if (Math.abs(nextRotationalSpeed) < Math.abs(rotationalFriction)) {
	      nextRotationalSpeed = 0.0;
	    } else {
	      nextRotationalSpeed -= rotationalFriction;
	    }
	    return new SimulatorDriveSubsystem.SpeedPair(nextLinearSpeed, nextRotationalSpeed);
	  }

}
