// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.PoseEstimator;

public class SetTrajectoryField2d extends CommandBase {
  private final PoseEstimator poseEstimator;
  private final Trajectory trajectory;
  /** Creates a new SetTrajectoryField2d. */
  public SetTrajectoryField2d(PoseEstimator poseEstimator, Trajectory trajectory) {
    this.poseEstimator = poseEstimator;
    this.trajectory = trajectory;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(poseEstimator);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    poseEstimator.setTrajectoryField2d(trajectory);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}