package frc.robot;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import frc.robot.util.Field2d;

public class SimulationState {
	private static final double MAX_ACCEL = 3.0; // That's pretty good accel!
	private static final double MAX_SPEED = 10.0; // m/s: really fast!
	private static final double MAX_ROTATIONAL_SPEED = 2.0 * Math.PI; // seems pretty fast!
	private static final double MAX_ROTATIONAL_ACCEL = 1.0;

    private final Translation2d target = new Translation2d(8.0, 4.0);

    public static class SpeedPair {
	    public final double linear;
	    public final double rotational;
	
	    public SpeedPair(double linear, double rotational) {
	      this.linear = linear;
	      this.rotational = rotational;
	    }
	  }

	private static final Field2d field2d = new Field2d();
    private double robotLinearSpeed;
    private double robotRotationalSpeed;
    private long lastNanos;
	private static final double ROTATIONAL_FRICTION_COEFFICIENT = 0.1;
	private static final double BRAKING_ROTATIONAL_FRICTION_COEFFICIENT = 1.0;
	private static final double LINEAR_FRICTION_COEFFICIENT = 0.1;
	private static final double BRAKING_LINEAR_FRICTION_COEFFICIENT = 1.0;
	private static final double ONE_BILLION = 1000000000.0;
	private boolean isBraking;

	double getLinearFriction() {
		if (isBraking) {
			return BRAKING_LINEAR_FRICTION_COEFFICIENT;
		} else {
			return LINEAR_FRICTION_COEFFICIENT;
		}
	}

	double getRotationalFriction() {
		if (isBraking) {
			return BRAKING_ROTATIONAL_FRICTION_COEFFICIENT;
		} else {
			return ROTATIONAL_FRICTION_COEFFICIENT;
		}
	}

	public void setBraking(boolean isBraking) {
		this.isBraking = isBraking;
	}

    public SimulationState() {
		lastNanos = System.nanoTime();
		reset();
    }

    public void reset() {
        field2d.setRobotPose(new Pose2d());
        robotLinearSpeed = 0.0;
		robotRotationalSpeed = 0.0;
		isBraking = false;
    }

    public Pose2d getRobotPosition() {
        return field2d.getRobotPose();
    }

    public void setRobotPosition(Pose2d position) {
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

    public Translation2d getTargetPosition() {
        return target;
    }

	public void updateSimulationState(double linearAcceleration, double rotationalAcceleration) {
	    long now = System.nanoTime();
	    long intervalNanos = now - lastNanos;
	    lastNanos = now;
	
	    Pose2d currentPosition = getRobotPosition();
	    double intervalSeconds = intervalNanos / SimulationState.ONE_BILLION;
	    // Update the position
	    double currentLinearSpeed = getRobotLinearSpeed();
	    double currentRotationalSpeed = getRobotRotationalSpeed();
	    Pose2d nextPosition = calculateNextPosition(currentPosition, currentLinearSpeed, currentRotationalSpeed, intervalSeconds);
	    setRobotPosition(nextPosition);
	    SpeedPair nextVelocity = calculateNextVelocity(currentLinearSpeed, currentRotationalSpeed, intervalSeconds, linearAcceleration, rotationalAcceleration);
	    setRobotLinearSpeed(clamp(nextVelocity.linear, -MAX_SPEED, MAX_SPEED));
	    setRobotRotationalSpeed(clamp(nextVelocity.rotational, -MAX_ROTATIONAL_SPEED, MAX_ROTATIONAL_SPEED));
	  }

	public Pose2d calculateNextPosition(Pose2d currentPosition, double linearSpeed, double rotationalSpeed, double intervalSeconds) {
	    Translation2d currentVelocity = new Translation2d(linearSpeed, 0.0).rotateBy(currentPosition.getRotation());
	    Translation2d nextTranslation = currentPosition.getTranslation().plus(currentVelocity.times(intervalSeconds));
	    Rotation2d nextRotation = currentPosition.getRotation().plus(new Rotation2d(rotationalSpeed * intervalSeconds));
	    return new Pose2d(nextTranslation, nextRotation);
	  }

	public SimulationState.SpeedPair calculateNextVelocity(double currentLinearSpeed, double currentRotationalSpeed, double intervalSeconds, double linearAcceleration, double rotationalAcceleration) {
		linearAcceleration = clamp(linearAcceleration, -MAX_ACCEL, MAX_ACCEL);
		rotationalAcceleration = clamp(rotationalAcceleration, -MAX_ROTATIONAL_ACCEL, MAX_ROTATIONAL_ACCEL);
	    double nextLinearSpeed = currentLinearSpeed + linearAcceleration * intervalSeconds;
	    double linearFriction = getLinearFriction() * intervalSeconds * Math.signum(nextLinearSpeed);
	    if (Math.abs(nextLinearSpeed) < Math.abs(linearFriction)) {
	      nextLinearSpeed = 0.0;
	    } else {
	      nextLinearSpeed -= linearFriction;
	    }
	    double nextRotationalSpeed = currentRotationalSpeed + rotationalAcceleration * intervalSeconds;
	    double rotationalFriction = getRotationalFriction() * intervalSeconds * Math.signum(nextRotationalSpeed);
	    if (Math.abs(nextRotationalSpeed) < Math.abs(rotationalFriction)) {
	      nextRotationalSpeed = 0.0;
	    } else {
	      nextRotationalSpeed -= rotationalFriction;
	    }
	    return new SimulationState.SpeedPair(nextLinearSpeed, nextRotationalSpeed);
	  }


	    // Clamp to value to the rangle [mix,max]
  public static double clamp(double value, double min, double max) {
    if (value < min) {
      return min;
    } else if (value > max) {
      return max;
    } else {
      return value;
    }
  }


}
