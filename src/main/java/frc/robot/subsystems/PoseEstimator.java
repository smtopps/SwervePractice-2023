// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Collections;
import java.util.List;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.SwerveConstants;

public class PoseEstimator extends SubsystemBase {
  private final SwerveSubsystem swerveSubsystem;
  private final Pigeon2Subsystem pigeon2Subsystem;
  
  private final PhotonCamera photonCamera = new PhotonCamera("OV5647");;

  // Physical location of the camera on the robot, relative to the center of the robot.
  private final Transform3d cameraToRobot = new Transform3d(
    new Translation3d(Units.inchesToMeters(-12.0), 0.0, Units.inchesToMeters(-5.25)),
    new Rotation3d());

  // Ordered list of target poses by ID (WPILib is adding some functionality for this)
  private final List<Pose3d> targetPoses = Collections.unmodifiableList(List.of(
    new Pose3d(Units.feetToMeters(54.0), Units.feetToMeters(13.5), Units.inchesToMeters(6.0), new Rotation3d(0.0, 0.0, Units.degreesToRadians(180.0))),
    new Pose3d(Units.feetToMeters(0.0), Units.feetToMeters(13.5), Units.inchesToMeters(6.0), new Rotation3d(0.0, 0.0, 0.0))));

  // Kalman Filter Configuration. These can be "tuned-to-taste" based on how much you trust your various sensors. 
  // Smaller numbers will cause the filter to "trust" the estimate from that particular component more than the others. 
  // This in turn means the particualr component will have a stronger influence on the final pose estimate.
  private final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5));
  private final Matrix<N3, N1> visionMeasurementStdDevs = VecBuilder.fill(0.02, 0.02, Units.degreesToRadians(5));
  private static SwerveDrivePoseEstimator poseEstimator;
  
  private final Field2d field2d = new Field2d();
  
  /** Creates a new PoseEstimator3d. */
  public PoseEstimator(SwerveSubsystem swerveSubsystem, Pigeon2Subsystem pigeon2Subsystem) {
    this.swerveSubsystem = swerveSubsystem;
    this.pigeon2Subsystem = pigeon2Subsystem;

    poseEstimator = new SwerveDrivePoseEstimator(
      SwerveConstants.KINEMATICS, 
      pigeon2Subsystem.getGyroRotation(), 
      swerveSubsystem.getPositions(), 
      new Pose2d(new Translation2d(0, 0), new Rotation2d(0.0)), 
      stateStdDevs, 
      visionMeasurementStdDevs);

    SmartDashboard.putData("Field", field2d);
  }

  @Override
  public void periodic() {
    // Update pose estimator with visible targets
    // latest pipeline result
    PhotonPipelineResult res = photonCamera.getLatestResult();
    SmartDashboard.putBoolean("Camera Has Target", res.hasTargets());
    if(res.hasTargets()) {
      // if there are targets record the image capture time
      double imageCaptureTime = Timer.getFPGATimestamp() - (res.getLatencyMillis() / 1000.0);
      PhotonTrackedTarget bestTarget = res.getBestTarget();
      int fiducialId = bestTarget.getFiducialId();
      if(fiducialId >= 0 && fiducialId < targetPoses.size() && bestTarget.getPoseAmbiguity() < 0.2) {
        Pose3d targetPose = targetPoses.get(fiducialId);
        Transform3d camToTargetTrans = bestTarget.getBestCameraToTarget();
        Pose3d camPose = targetPose.transformBy(camToTargetTrans.inverse());
        //Pose3d camPose = targetPose.transformBy(camToTargetTrans);
        Pose3d robotPose = camPose.transformBy(cameraToRobot);
        poseEstimator.addVisionMeasurement(robotPose.toPose2d(), imageCaptureTime);
        //SmartDashboard.putString("Pose3d", robotPose.toString());
      }
    }

    poseEstimator.updateWithTime(Timer.getFPGATimestamp(), pigeon2Subsystem.getGyroRotation(), swerveSubsystem.getPositions());
    field2d.setRobotPose(poseEstimator.getEstimatedPosition());
    SmartDashboard.putString("Pose", poseEstimator.getEstimatedPosition().toString());
    // This method will be called once per scheduler run
  }

  public static Pose2d getCurrentPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public double getPoseX() {
    return poseEstimator.getEstimatedPosition().getX();
  }

  public double getPoseY() {
    return poseEstimator.getEstimatedPosition().getY();
  }

  public double getPoseTheta() {
    return poseEstimator.getEstimatedPosition().getRotation().getDegrees();
  }

  public Rotation2d getPoseRotation() {
    return poseEstimator.getEstimatedPosition().getRotation();
  }

  public void setPose(Pose2d pose) {
    poseEstimator.resetPosition(pigeon2Subsystem.getGyroscopeRotation(), swerveSubsystem.getPositions(), pose);
  }

  public void setTrajectoryField2d(Trajectory trajectory) {
    field2d.getObject("traj").setTrajectory(trajectory);
  }
}
