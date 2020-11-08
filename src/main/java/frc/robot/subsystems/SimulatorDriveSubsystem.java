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

    Pose2d pose = simulationState.getRobotPosition();
    double intervalSeconds = intervalNanos / ONE_BILLION;
    // Calculate the acceleration
    Translation2d linearAccelerationVector = new Translation2d(linearAcceleration, 0.0).rotateBy(pose.getRotation());
    Translation2d frictionAccelerationVector = new Translation2d(LINEAR_FRICTION_COEFFICIENT, 0.0).rotateBy(pose.getRotation());

    // Update the position
    Pose2d velocity = simulationState.getRobotVelocity();
    Translation2d nextTranslation = pose.getTranslation().plus(velocity.getTranslation().times(intervalSeconds));
    Rotation2d nextRotation = pose.getRotation().plus(velocity.getRotation().times(intervalSeconds));
    simulationState.setPosition(new Pose2d(nextTranslation, nextRotation));
    // Update the velocity
    Translation2d nextTranslationVelocity = velocity.getTranslation().plus(linearAccelerationVector.times(intervalSeconds));
    if (nextTranslationVelocity.getNorm() > frictionAccelerationVector.getNorm()) {
      nextTranslationVelocity = nextTranslationVelocity.minus(frictionAccelerationVector);
    } else {
      nextTranslationVelocity = new Translation2d(); // zero
    }

    Rotation2d nextRotationalVelocity = velocity.getRotation().plus(new Rotation2d(rotationalAcceleration * intervalSeconds));
    if (Math.abs(nextRotationalVelocity.getRadians()) < ROTATIONAL_FRICTION_COEFFICIENT) {
      nextRotationalVelocity = new Rotation2d(); //zero
    } else {
      double directedRotationalFriction = Math.signum(nextRotationalVelocity.getRadians()) * ROTATIONAL_FRICTION_COEFFICIENT;
      nextRotationalVelocity = nextRotationalVelocity.minus(new Rotation2d(directedRotationalFriction));
    }

    simulationState.setVelocity(new Pose2d(nextTranslationVelocity, nextRotationalVelocity));
    //logger.error("interval: "+intervalSeconds + " accel: "+fmtTranslation(linearAccelerationVector) + " vel: "+fmtTranslation(nextTranslationVelocity));
  }


  private String fmtTranslation(Translation2d trans) {
    return "("+trans.getX()+", "+trans.getY()+")";
  }
}
