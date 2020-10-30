/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {
  private final CANSparkMax leftMotor = new CANSparkMax(1, MotorType.kBrushless);
  private final CANSparkMax rightMotor = new CANSparkMax(2, MotorType.kBrushless);
  private double leftThrottle;
  private double rightThrottle;

  /**
   * Creates a new DriveSubsystem.
   */
  public DriveSubsystem() {
    reset();
  }

  public void reset() {
    setThrottle(0.0, 0.0);
  }

  public void setThrottle(double left, double right) {
    leftThrottle = left;
    rightThrottle = right;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    leftMotor.set(leftThrottle);
    rightMotor.set(rightThrottle);
  }
}
