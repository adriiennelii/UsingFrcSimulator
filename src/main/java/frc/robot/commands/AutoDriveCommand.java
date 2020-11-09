/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.sensors.MagicPositionSensor;
import frc.robot.sensors.MagicTargetSensor;
import frc.robot.subsystems.SimulatorDriveSubsystem;

public class AutoDriveCommand extends CommandBase {
  private static final Logger logger = LogManager.getLogger(AutoDriveCommand.class);
  private final SimulatorDriveSubsystem driveSubsystem;
  private final MagicPositionSensor positionSensor;
  private final MagicTargetSensor targetSensor;
  private boolean isDone = false;
  private static final double DISTANCE_CLOSE_ENOUGH = 0.1;
  private static final double HEADING_CLOSE_ENOUGH = 1.0;

  private static final double kP_linear = 0.15;
  private static final double kD_linear = 0.4;
  private static final double kP_rotational = 0.2;
  private static final double kD_rotational = 1.9;

  public AutoDriveCommand(SimulatorDriveSubsystem driveSubsystem, MagicPositionSensor positionSensor, MagicTargetSensor targetSensor) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.driveSubsystem = driveSubsystem;
    this.positionSensor = positionSensor;
    this.targetSensor = targetSensor;
    addRequirements(driveSubsystem);
  }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    driveSubsystem.reset();
    isDone = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double distance = targetSensor.getDistanceToTarget();
    double heading = targetSensor.getHeadingToTarget();
    double speed = positionSensor.getRobotLinearSpeed();
    double rotationSpeed = positionSensor.getRobotRotationalSpeed();
    // Align with the target position

    if (distance < DISTANCE_CLOSE_ENOUGH) {
      // Made it!      
      isDone = true;
    } else if (Math.abs(heading) < HEADING_CLOSE_ENOUGH) {
      // drive towards the target!
      driveSubsystem.setAcceleration(kP_linear * distance - kD_linear * speed, 0.0);
    } else {
      driveSubsystem.setAcceleration(0.0, kP_rotational * heading - kD_rotational * rotationSpeed);
    }

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    driveSubsystem.setBrakes(true);
    driveSubsystem.setAcceleration(0.0, 0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
