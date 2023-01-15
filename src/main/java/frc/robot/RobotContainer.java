
package frc.robot;

import java.util.HashMap;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.PIDConstants;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Auto.TestPath;
import frc.robot.commands.ChangeMaxSpeed;
import frc.robot.commands.DriveToLoadingStation;
import frc.robot.commands.DriveWithJoysticks;
import frc.robot.commands.LockDrive;
import frc.robot.commands.ToggleFieldRelative;
import frc.robot.commands.SetPose;
import frc.robot.subsystems.CANdleSubsystem;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Pigeon2Subsystem;
import frc.robot.subsystems.SwerveSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public static double maxSpeed = Constants.DRIVE_SPEED;
  public static boolean fieldRelative = true;
  private final XboxController driverController = new XboxController(Constants.DRIVER_CONTROLLER);

  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  private final Pigeon2Subsystem pigeon2Subsystem = new Pigeon2Subsystem();
  private final Limelight limelight = new Limelight(swerveSubsystem, pigeon2Subsystem);
  private final CANdleSubsystem candleSubsystem = new CANdleSubsystem();

  //Auto Stuff
  private final TestPath testPath = new TestPath(swerveSubsystem, limelight);
  SendableChooser<String> chooser = new SendableChooser<>();

  //On The Fly Trajectory Stuff
  public static PathPlannerTrajectory trajectory;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new DriveWithJoysticks(
      swerveSubsystem,
      limelight,
      () -> -driverController.getLeftX(),
      () -> -driverController.getLeftY(),
      () -> -driverController.getRightX(),
      () -> fieldRelative,
      () -> maxSpeed));
    // Configure the button bindings
    configureButtonBindings();

    chooser.setDefaultOption("Blue1", "Blue1");
    chooser.addOption("Blue2", "Blue2");
    SmartDashboard.putData(chooser);
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    new JoystickButton(driverController, 8).onTrue(new SetPose(limelight, new Pose2d(0.0, 0.0, new Rotation2d(0.0))));
    new JoystickButton(driverController, 6).whileTrue(new ChangeMaxSpeed(Constants.BOOST_SPEED));
    new JoystickButton(driverController, 5).whileTrue(new ChangeMaxSpeed(Constants.PERCISION_SPEED));
    new JoystickButton(driverController, 3).onTrue(new ToggleFieldRelative());
    new JoystickButton(driverController, XboxController.Button.kA.value).whileTrue(new LockDrive(swerveSubsystem));
    new JoystickButton(driverController, 4)
      .onTrue(new DriveToLoadingStation(swerveSubsystem, limelight, candleSubsystem))
      .onFalse(new InstantCommand(() -> {
        if(swerveSubsystem.getCurrentCommand() != null){
          swerveSubsystem.getCurrentCommand().cancel();
          candleSubsystem.setDefult();
        }
      }));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous

    // Create the HashMap to run Commands for Auto.
    HashMap<String, Command> eventMap = new HashMap<>();
    eventMap.put("marker1", new PrintCommand("1"));
    eventMap.put("marker2", new PrintCommand("2"));

    // Create the AutoBuilder.
    SwerveAutoBuilder autoBuilder = new SwerveAutoBuilder(
      limelight::getPose, // Pose2d supplier
      limelight::setPose, // Pose2d consumer, used to reset odometry at the beginning of auto
      Constants.SwerveConstants.KINEMATICS, // SwerveDriveKinematics
      new PIDConstants(Constants.AutoConstants.kPXController, 0.0, 0.0), // PID constants to correct for translation error (used to create the X and Y PID controllers)
      new PIDConstants(Constants.AutoConstants.kPThetaController, 0.0, 0.0), // PID constants to correct for rotation error (used to create the rotation controller)
      swerveSubsystem::setModuleStates, // Module states consumer used to output to the drive subsystem
      eventMap,
      swerveSubsystem // The drive subsystem. Used to properly set the requirements of path following commands
    );

    List<PathPlannerTrajectory> trajectories;
    if(chooser.getSelected() == "Blue1") {
      trajectories = PathPlanner.loadPathGroup(
        chooser.getSelected(),
        new PathConstraints(2, 2),
        new PathConstraints(2, 2));
      return autoBuilder.fullAuto(trajectories);
    }else{
      return new PrintCommand("Nothing");
    }
  }
}
