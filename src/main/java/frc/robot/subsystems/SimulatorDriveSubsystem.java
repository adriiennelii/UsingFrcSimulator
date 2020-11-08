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

  public static class SpeedPair {
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
    updateSimulationState(simulationState, linearAcceleration, rotationalAcceleration);
  }

  public static void updateSimulationState(SimulationState simulationState, double linearAcceleration, double rotationalAcceleration) {
    long now = System.nanoTime();
    long intervalNanos = now - simulationState.lastNanos;
    simulationState.lastNanos = now;

    Pose2d currentPosition = simulationState.getRobotPosition();
    double intervalSeconds = intervalNanos / ONE_BILLION;
    // Update the position
    double currentLinearSpeed = simulationState.getRobotLinearSpeed();
    double currentRotationalSpeed = simulationState.getRobotRotationalSpeed();
    Pose2d nextPosition = SimulationState.calculateNextPosition(currentPosition, currentLinearSpeed, currentRotationalSpeed, intervalSeconds);
    simulationState.setPosition(nextPosition);
    SpeedPair nextVelocity = SimulationState.calculateNextVelocity(currentLinearSpeed, currentRotationalSpeed, intervalSeconds, linearAcceleration, rotationalAcceleration);
    simulationState.setRobotLinearSpeed(nextVelocity.linear);
    simulationState.setRobotRotationalSpeed(nextVelocity.rotational);
  }
}
