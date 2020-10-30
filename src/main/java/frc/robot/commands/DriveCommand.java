/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class DriveCommand extends CommandBase {
  private final DriveSubsystem driveSubsystem;
  private final Supplier<Double> leftThrottle;
  private final Supplier<Double> rightThrottle;
  /**
   * Creates a new DriveCommand.
   */
  public DriveCommand(DriveSubsystem driveSubsystem, Supplier<Double> leftThrottle, Supplier<Double> rightThrottle) {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(driveSubsystem);
    this.driveSubsystem = driveSubsystem;
    this.leftThrottle = leftThrottle;
    this.rightThrottle = rightThrottle;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.reset();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    driveSubsystem.setThrottle(leftThrottle.get(), rightThrottle.get());
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
