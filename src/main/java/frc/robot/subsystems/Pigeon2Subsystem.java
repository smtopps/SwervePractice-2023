// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2.AxisDirection;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Pigeon2Subsystem extends SubsystemBase {
  /** Creates a new PigeonSubsystemTwo. */
  private final Pigeon2 pigeon2 = new Pigeon2(Constants.DRIVETRAIN_PIGEON_ID, "canivore");
  
  public Pigeon2Subsystem() {
    pigeon2.configMountPose(AxisDirection.PositiveY, AxisDirection.PositiveZ);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void zeroGyroscope() {
    pigeon2.setYaw(0);
  }

  public Rotation2d getGyroRotation() {
    return Rotation2d.fromDegrees(pigeon2.getYaw());
  }

  public Rotation2d getGyroscopeRotation() {
    return Rotation2d.fromDegrees(pigeon2.getYaw());
  }
}

