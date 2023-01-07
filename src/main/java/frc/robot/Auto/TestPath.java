// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.SetPose;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestPath extends SequentialCommandGroup {
  /** Creates a new TestPath. */
  public TestPath(SwerveSubsystem swerveSubsystem, PoseEstimator poseEstimator) {
    PathPlannerTrajectory trajectory1 = swerveSubsystem.loadTrajectoryFromFile("TestPath", 2, 2);
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetPose(poseEstimator, trajectory1.getInitialPose()),
      swerveSubsystem.createCommandForTrajectory(trajectory1)
    );
  }
}
