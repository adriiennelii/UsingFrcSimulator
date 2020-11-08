/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.MagicPositionSensor;
import frc.robot.subsystems.SimulatorDriveSubsystem;

public class AutoDriveCommand extends CommandBase {
  private final SimulatorDriveSubsystem driveSubsystem;
  private final MagicPositionSensor positionSensor;

  // Where are we going? (4.0, 4.0)! When? REAL SOON!
  private  final Pose2d target = new Pose2d(new Translation2d(4.0, 4.0), new Rotation2d(0.0));

  public AutoDriveCommand(SimulatorDriveSubsystem driveSubsystem, MagicPositionSensor positionSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.positionSensor = positionSensor;
    addRequirements(driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Get vector to target
    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.reset();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
