// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Auto;

import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.SwerveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PathGroup extends SequentialCommandGroup {
  /** Creates a new PathGroup. */
  public PathGroup(SwerveSubsystem swerveSubsystem, Limelight limelight) {
    List<PathPlannerTrajectory> trajectories = PathPlanner.loadPathGroup(
      "PathGroup",
      new PathConstraints(1, 1),
      new PathConstraints(3, 2));
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      swerveSubsystem.createCommandForTrajectory(trajectories.get(0)),
      new WaitCommand(2),
      swerveSubsystem.createCommandForTrajectory(trajectories.get(1))
    );
  }
}
