/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.SimulationState;

public class SimulatorDriveSubsystem extends SubsystemBase {
  private final SimulationState simulationState;

  private double linearAcceleration;
  private double rotationalAcceleration;
  private boolean isBraking;

  /**
   */
  public SimulatorDriveSubsystem(SimulationState simulationState) {
    this.simulationState = simulationState;
    reset();
  }

  public void setAcceleration(double linearAcceleration, double rotationalAcceleration) {
    this.linearAcceleration = linearAcceleration;
    this.rotationalAcceleration = rotationalAcceleration;
  }

  public void setBrakes(boolean isBraking) {
    this.isBraking = isBraking;
  }

  public void reset() {
    linearAcceleration = 0.0;
    rotationalAcceleration = 0.0;
    isBraking = false;
  }

  @Override
  public void periodic() {
    simulationState.setBraking(isBraking);
    simulationState.updateSimulationState(linearAcceleration, rotationalAcceleration);
  }
}
