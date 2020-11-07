/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Transform2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.ControlState;
import frc.robot.util.Field2d;

import java.util.function.Supplier;

public class SimulatorDriveSubsystem extends SubsystemBase {
  private static final double ONE_BILLION = 1000000000.0;
  private final Supplier<ControlState> controlSupplier;
  private final Field2d field2d = new Field2d();
  private Pose2d velocity;
  private long lastNanos = System.nanoTime();
  /**
   * Creates a new SimulatorDriveSubsystem.
   */
  public SimulatorDriveSubsystem(Supplier<ControlState> controlSupplier) {
    this.controlSupplier = controlSupplier;
    velocity = new Pose2d(new Translation2d(1.0, 0.0), new Rotation2d());
    
  }


  @Override
  public void periodic() {
    long intervalNanos = System.nanoTime() - lastNanos;
    //ControlState state = controlSupplier.get();
    Pose2d pose = field2d.getRobotPose();
    double intervalSeconds = intervalNanos / ONE_BILLION;
    Translation2d nextTranslation = pose.getTranslation().plus(velocity.getTranslation().times(intervalSeconds));
    Rotation2d nextRotation = pose.getRotation().plus(velocity.getRotation().times(intervalSeconds));
    field2d.setRobotPose(new Pose2d(nextTranslation, nextRotation));
  }
}
