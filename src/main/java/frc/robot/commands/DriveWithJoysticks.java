// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.SwerveConstants;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

public class DriveWithJoysticks extends CommandBase {
  private final SwerveSubsystem swerveSubsystem;
  private final PoseEstimator poseEstimator;

  private final DoubleSupplier translationX;
  private final DoubleSupplier translationY;
  private final DoubleSupplier rotation;
  private final BooleanSupplier relative;
  private final DoubleSupplier maxSpeed;

  private final SlewRateLimiter xLimiter, yLimiter, turnLimiter;

  /** Creates a new DriveWithJoysticks. */
  public DriveWithJoysticks(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator, DoubleSupplier translationX, DoubleSupplier translationY, DoubleSupplier rotation, BooleanSupplier relative, DoubleSupplier maxSpeed) {
    this.swerveSubsystem = swerveSubsystem;
    this.poseEstimator = poseEstimator;
    this.translationX = translationX;
    this.translationY = translationY;
    this.rotation = rotation;
    this.relative = relative;
    this.maxSpeed = maxSpeed;
    this.xLimiter = new SlewRateLimiter(2.0);
    this.yLimiter = new SlewRateLimiter(2.0);
    this.turnLimiter = new SlewRateLimiter(2.0);
    addRequirements(swerveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(relative.getAsBoolean()) {
      swerveSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
      modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
      modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND,
      poseEstimator.getPoseRotation()));
    } else {
      swerveSubsystem.drive(new ChassisSpeeds(
        modifyAxis(translationY.getAsDouble(), maxSpeed.getAsDouble(), yLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(translationX.getAsDouble(), maxSpeed.getAsDouble(), xLimiter) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
        modifyAxis(rotation.getAsDouble(), maxSpeed.getAsDouble(), turnLimiter) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    swerveSubsystem.stop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

  public double modifyAxis(double value, double speedModifyer, SlewRateLimiter limiter) {
    value = MathUtil.applyDeadband(value, 0.02);
    value = Math.copySign(value * value, value);
    value = value*speedModifyer;
    value = limiter.calculate(value);
    if(Math.abs(value)*Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND <= Constants.SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND*0.01){
      value = 0.0;
    }

    return value;
  }
}