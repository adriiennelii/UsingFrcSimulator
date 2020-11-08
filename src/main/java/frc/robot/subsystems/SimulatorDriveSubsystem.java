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
  private static final double ONE_BILLION = 1000000000.0;
  private static final double LINEAR_FRICTION_COEFFICIENT = 0.1;
  private static final double ROTATIONAL_FRICTION_COEFFICIENT = 0.1;
  private final SimulationState simulationState;
  private long lastNanos = System.nanoTime();

  private double linearAcceleration;
  private double rotationalAcceleration;

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
    Pose2d currentVelocity = simulationState.getRobotVelocity();
    Pose2d nextPosition = calculateNextPosition(currentPosition, currentVelocity, intervalSeconds);
    simulationState.setPosition(nextPosition);
    Pose2d nextVelocity = calculateNextVelocity(currentPosition.getRotation(), currentVelocity, intervalSeconds, linearAcceleration, rotationalAcceleration);
    simulationState.setVelocity(nextVelocity);
  }

  static Pose2d calculateNextPosition(Pose2d currentPosition, Pose2d currentVelocity, double intervalSeconds) {
    Translation2d nextTranslation = currentPosition.getTranslation().plus(currentVelocity.getTranslation().times(intervalSeconds));
    Rotation2d nextRotation = currentPosition.getRotation().plus(currentVelocity.getRotation().times(intervalSeconds));
    return new Pose2d(nextTranslation, nextRotation);
  }

  static Pose2d calculateNextVelocity(Rotation2d currentRotation, Pose2d currentVelocity, double intervalSeconds, double linearAcceleration, double rotationalAcceleration) {
    // Calculate the acceleration
    Translation2d linearAccelerationVector = new Translation2d(linearAcceleration, 0.0).rotateBy(currentRotation);
    Translation2d frictionAccelerationVector = new Translation2d(LINEAR_FRICTION_COEFFICIENT, 0.0).rotateBy(currentRotation);
    // Update the velocity
    Translation2d nextTranslationVelocity = currentVelocity.getTranslation().plus(linearAccelerationVector.times(intervalSeconds));
    if (nextTranslationVelocity.getNorm() > frictionAccelerationVector.getNorm()) {
      nextTranslationVelocity = nextTranslationVelocity.minus(frictionAccelerationVector);
    } else {
      nextTranslationVelocity = new Translation2d(); // zero
    }

    Rotation2d nextRotationalVelocity = currentVelocity.getRotation().plus(new Rotation2d(rotationalAcceleration * intervalSeconds));
    if (Math.abs(nextRotationalVelocity.getRadians()) < ROTATIONAL_FRICTION_COEFFICIENT) {
      nextRotationalVelocity = new Rotation2d(); //zero
    } else {
      double directedRotationalFriction = Math.signum(nextRotationalVelocity.getRadians()) * ROTATIONAL_FRICTION_COEFFICIENT;
      nextRotationalVelocity = nextRotationalVelocity.minus(new Rotation2d(directedRotationalFriction));
    }

    return new Pose2d(nextTranslationVelocity, nextRotationalVelocity);

  }

}
