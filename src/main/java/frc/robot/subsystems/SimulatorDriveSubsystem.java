/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.AccelerationState;
import frc.robot.util.Field2d;

import java.util.function.Supplier;

import org.apache.logging.log4j.LogManager;
import org.apache.logging.log4j.Logger;

public class SimulatorDriveSubsystem extends SubsystemBase {
  private static final Logger logger = LogManager.getLogger(SimulatorDriveSubsystem.class);
  private static final double ONE_BILLION = 1000000000.0;
  private final Supplier<AccelerationState> controlSupplier;
  private final Field2d field2d = new Field2d();
  private Pose2d velocity;
  private long lastNanos = System.nanoTime();
  /**
   * Creates a new SimulatorDriveSubsystem.
   */
  public SimulatorDriveSubsystem(Supplier<AccelerationState> controlSupplier) {
    this.controlSupplier = controlSupplier;
    velocity = new Pose2d(new Translation2d(), new Rotation2d());
    
  }

  @Override
  public void periodic() {
    long now = System.nanoTime();
    long intervalNanos = now - lastNanos;
    lastNanos = now;

    AccelerationState state = controlSupplier.get();
    Pose2d pose = field2d.getRobotPose();
    double intervalSeconds = intervalNanos / ONE_BILLION;
    // Calculate the acceleration
    Translation2d linearAccelerationVector = new Translation2d(0.0, state.linearAcceleration).rotateBy(pose.getRotation());

    // Update the position
    Translation2d nextTranslation = pose.getTranslation().plus(velocity.getTranslation().times(intervalSeconds));
    Rotation2d nextRotation = pose.getRotation().plus(velocity.getRotation().times(intervalSeconds));
    field2d.setRobotPose(new Pose2d(nextTranslation, nextRotation));
    // Update the velocity
    Translation2d nextTranslationVelocity = velocity.getTranslation().plus(linearAccelerationVector.times(intervalSeconds));
    Rotation2d nextRotationalVelocity = velocity.getRotation().plus(new Rotation2d(state.rotationalAcceleration * intervalSeconds));
    velocity = new Pose2d(nextTranslationVelocity, nextRotationalVelocity);
    logger.error("interval: "+intervalSeconds + " accel: "+fmtTranslation(linearAccelerationVector) + " vel: "+fmtTranslation(nextTranslationVelocity));
  }


  private String fmtTranslation(Translation2d trans) {
    return "("+trans.getX()+", "+trans.getY()+")";
  }
}
