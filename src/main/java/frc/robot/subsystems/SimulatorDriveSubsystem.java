/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SimulationState;

public class SimulatorDriveSubsystem extends SubsystemBase {
  private static final Logger logger = LogManager.getLogger(SimulatorDriveSubsystem.class);
  public static final double ONE_BILLION = 1000000000.0;
  public static final double LINEAR_FRICTION_COEFFICIENT = 0.1;
  public static final double ROTATIONAL_FRICTION_COEFFICIENT = 0.1;
  private final SimulationState simulationState;
  private long lastNanos = System.nanoTime();

  private double linearAcceleration;
  private double rotationalAcceleration;

  static class SpeedPair {
    public final double linear;
    public final double rotational;

    public SpeedPair(double linear, double rotational) {
      this.linear = linear;
      this.rotational = rotational;
    }
  }

  /**
   * Creates a new SimulatorDriveSubsystem.
   */
  public SimulatorDriveSubsystem(SimulationState simulationState) {
    this.simulationState = simulationState;
    linearAcceleration = 0.0;
    rotationalAcceleration = 0.0;    
  }

  public void setAcceleration(double linearAcceleration, double rotationalAcceleration) {
    this.linearAcceleration = linearAcceleration;
    this.rotationalAcceleration = rotationalAcceleration;
  }

  public void reset() {
    linearAcceleration = 0.0;
    rotationalAcceleration = 0.0;
  }

  @Override
  public void periodic() {
    long now = System.nanoTime();
    long intervalNanos = now - lastNanos;
    lastNanos = now;

    Pose2d currentPosition = simulationState.getRobotPosition();
    double intervalSeconds = intervalNanos / ONE_BILLION;
    // Update the position
    double currentLinearSpeed = simulationState.getRobotLinearSpeed();
    double currentRotationalSpeed = simulationState.getRobotRotationalSpeed();
    Pose2d nextPosition = calculateNextPosition(currentPosition, currentLinearSpeed, currentRotationalSpeed, intervalSeconds);
    simulationState.setPosition(nextPosition);
    SpeedPair nextVelocity = calculateNextVelocity(currentLinearSpeed, currentRotationalSpeed, intervalSeconds, linearAcceleration, rotationalAcceleration);
    simulationState.setRobotLinearSpeed(nextVelocity.linear);
    simulationState.setRobotRotationalSpeed(nextVelocity.rotational);
  }

  public static Pose2d calculateNextPosition(Pose2d currentPosition, double linearSpeed, double rotationalSpeed, double intervalSeconds) {
    Translation2d currentVelocity = new Translation2d(linearSpeed, 0.0).rotateBy(currentPosition.getRotation());
    Translation2d nextTranslation = currentPosition.getTranslation().plus(currentVelocity.times(intervalSeconds));
    Rotation2d nextRotation = currentPosition.getRotation().plus(new Rotation2d(rotationalSpeed * intervalSeconds));
    return new Pose2d(nextTranslation, nextRotation);
  }

  public static SpeedPair calculateNextVelocity(double currentLinearSpeed, double currentRotationalSpeed, double intervalSeconds, double linearAcceleration, double rotationalAcceleration) {
    double nextLinearSpeed = currentLinearSpeed + linearAcceleration * intervalSeconds;
    double linearFriction = LINEAR_FRICTION_COEFFICIENT * intervalSeconds * Math.signum(nextLinearSpeed);
    if (Math.abs(nextLinearSpeed) < Math.abs(linearFriction)) {
      nextLinearSpeed = 0.0;
    } else {
      nextLinearSpeed -= linearFriction;
    }
    double nextRotationalSpeed = currentRotationalSpeed + rotationalAcceleration * intervalSeconds;
    double rotationalFriction = ROTATIONAL_FRICTION_COEFFICIENT * intervalSeconds * Math.signum(nextRotationalSpeed);
    if (Math.abs(nextRotationalSpeed) < Math.abs(rotationalFriction)) {
      nextRotationalSpeed = 0.0;
    } else {
      nextRotationalSpeed -= rotationalFriction;
    }
    return new SpeedPair(nextLinearSpeed, nextRotationalSpeed);
  }

}
