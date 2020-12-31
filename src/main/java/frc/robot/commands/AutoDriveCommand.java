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
    //means distance from target
    double distanceToTarget = targetSensor.getDistanceToTarget();
    //means how much to rotate from robot to target (calculated by rotation of target minus rotatation of robot to get how much to rotate)
    double headingToTarget = targetSensor.getHeadingToTarget();
    //spped of robot
    double robotLinearSpeed = positionSensor.getRobotLinearSpeed();
    //speed of robot when rotating 
    double robotRotationSpeed = positionSensor.getRobotRotationalSpeed();

//if there's distance from target to robot and rotation of robot isn't aligned and aimed below target, then needs to rotate left while accelerating to be in line with target. 
if (distanceToTarget > 0.0 && headingToTarget > 0.000) {
  driveSubsystem.setAcceleration(0.5, 0.6);
//if there's distance from target to robot and rotation of robot isn't aligned and aimed above target, then needs to rotate right while accelerating to be in line with target. 
} else if (distanceToTarget > 0.0 && headingToTarget < 0.000) {
  driveSubsystem.setAcceleration(0.5, -0.6);
//if there's no distance from target to robot and headingToTarget = 0 , then brake. 
} else {
  driveSubsystem.setBrakes(true);
}

//when robot is 2 meters away from robot, then start braking to come to full stop around target 
if (distanceToTarget < 2.00000) {
  driveSubsystem.setBrakes(true);
  isDone = true;
}

System.out.println(distanceToTarget + " " + headingToTarget + " " + robotLinearSpeed + " " + robotRotationSpeed);

    // TODO: Make the robot navigate to the field position X=8.0, Y=4.0
    //
    // The robot should stop within 1 meter of this location.
    //
    //Decide what the robot should do, based on the information from the sensors (above).
    // Should the robot turn? Should it drive forwards or backwards? Should it put on the brakes?
    // 
    // To accelerate forward at 1 m/s^2:   driveSubsystem.setAcceleration(1.0, 0.0);
    // To accelerate backward at 1 m/s^2:  driveSubsystem.setAcceleration(-1.0, 0.0);
    // To accelerate rotating left at 1 radian/s^2:       driveSubsystem.setAcceleration(0.0, 1.0);
    // To accelerate rotating right at 2 radians/s^2:     driveSubsystem.setAcceleration(0.0, -2.0);
    // To put on the brakes:     driveSubsystem.setBrakes(true);
    // To take off the brakes:   driveSubsystem.setBrakes(false);
    //
    // When the robot is close enough to the target, set isDone = true

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return isDone;
  }
}
