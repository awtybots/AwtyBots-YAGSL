// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.Comparator;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

/** Aligns the robot to an algae piece detected by PhotonVision. */
public class AlignToAlgae extends Command {
  private final PIDController forwardController;
  private final PIDController strafeController;
  private final ProfiledPIDController thetaController;
  private final HolonomicDriveController holonomicController;

  private final PhotonCamera algaeCamera;
  private final SwerveSubsystem drivebase;

  private Timer dontSeeTargetTimer;
  private Timer stopTimer;

  public AlignToAlgae(SwerveSubsystem drivebase) {
    this.drivebase = drivebase;

    // Area acts as our stand-in "distance" metric for forward control.
    this.forwardController = new PIDController(Constants.AREA_ALIGNMENT_P_ALGAE, 0.0, 0.01);
    // Target yaw drives the lateral controller so the algae sits centered in view.
    this.strafeController = new PIDController(Constants.YAW_ALIGNMENT_P_ALGAE, 0.0, 0.01);

    this.thetaController =
        new ProfiledPIDController(
            Constants.ROT_ALIGNMENT_P_ALGAE,
            0.0,
            0.01,
            new TrapezoidProfile.Constraints(6.28, 3.14));
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.holonomicController =
        new HolonomicDriveController(forwardController, strafeController, thetaController);

    this.algaeCamera = new PhotonCamera("algae_cam");

    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTargetTimer = new Timer();
    this.dontSeeTargetTimer.start();

    forwardController.setTolerance(Constants.AREA_TOLERANCE_ALGAE);
    strafeController.setTolerance(Math.toRadians(Constants.YAW_TOLERANCE_ALGAE));
    thetaController.setTolerance(Math.toRadians(Constants.ROT_TOLERANCE_ALGAE));
  }

  @Override
  public void execute() {
    PhotonPipelineResult result = algaeCamera.getLatestResult();

    if (result.hasTargets() && !result.getTargets().isEmpty()) {
      dontSeeTargetTimer.reset();

      PhotonTrackedTarget bestTarget =
          result.getTargets().stream().max(Comparator.comparingDouble(PhotonTrackedTarget::getArea)).orElse(null);

      if (bestTarget == null) {
        drivebase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
        SmartDashboard.putBoolean("Algae_hasTarget", false);
        SmartDashboard.putNumber("Algae_poseValidTimer", stopTimer.get());
        return;
      }

      SmartDashboard.putBoolean("Algae_hasTarget", true);

      double latencySec = Math.max(0.0, Timer.getFPGATimestamp() - result.getTimestampSeconds());
      ChassisSpeeds robotSpeeds = drivebase.getRobotRelativeSpeeds();

      double measuredArea = bestTarget.getArea();
      double measuredYawDeg = bestTarget.getYaw();

      // Use drivetrain velocity to predict where the target appears after latency.
      double predictedArea =
          measuredArea + Constants.AREA_VELOCITY_COEFF_ALGAE * robotSpeeds.vxMetersPerSecond * latencySec;
      // Keep prediction within the camera's valid bounds.
      predictedArea = Math.max(0.0, predictedArea);

      double predictedYawDeg = measuredYawDeg + Math.toDegrees(robotSpeeds.omegaRadiansPerSecond * latencySec);

      Rotation2d currentHeading = drivebase.getPose().getRotation();
      Rotation2d predictedHeading = currentHeading.plus(Rotation2d.fromRadians(robotSpeeds.omegaRadiansPerSecond * latencySec));

      Pose2d currentMeasurement = new Pose2d(predictedArea, Math.toRadians(predictedYawDeg), predictedHeading);
      Pose2d goalPose =
          new Pose2d(
              Constants.AREA_SETPOINT_ALGAE,
              0.0,
              Rotation2d.fromDegrees(Constants.ROT_SETPOINT_ALGAE));

      ChassisSpeeds commandSpeeds =
          holonomicController.calculate(
              currentMeasurement,
              goalPose,
              0.0,
              Rotation2d.fromDegrees(Constants.ROT_SETPOINT_ALGAE));

      drivebase.drive(commandSpeeds);

      double areaError = Constants.AREA_SETPOINT_ALGAE - predictedArea;
      double yawErrorDeg = -predictedYawDeg;
      double rotErrorDeg =
          Rotation2d.fromDegrees(Constants.ROT_SETPOINT_ALGAE)
              .minus(predictedHeading)
              .getDegrees();

      boolean atArea = Math.abs(areaError) <= Constants.AREA_TOLERANCE_ALGAE;
      boolean atYaw = Math.abs(yawErrorDeg) <= Constants.YAW_TOLERANCE_ALGAE;
      boolean atRot = Math.abs(rotErrorDeg) <= Constants.ROT_TOLERANCE_ALGAE;

      if (!(atArea && atYaw && atRot)) {
        stopTimer.reset();
      }

      SmartDashboard.putNumber("Algae_TargetArea_meas", measuredArea);
      SmartDashboard.putNumber("Algae_TargetArea_pred", predictedArea);
      SmartDashboard.putNumber("Algae_TargetYaw_meas_deg", measuredYawDeg);
      SmartDashboard.putNumber("Algae_TargetYaw_pred_deg", predictedYawDeg);

      SmartDashboard.putNumber("Algae_err_Area", areaError);
      SmartDashboard.putNumber("Algae_err_Yaw_deg", yawErrorDeg);
      SmartDashboard.putNumber("Algae_err_Rot_deg", rotErrorDeg);

      SmartDashboard.putBoolean("Algae_atArea", atArea);
      SmartDashboard.putBoolean("Algae_atYaw", atYaw);
      SmartDashboard.putBoolean("Algae_atRot", atRot);
      SmartDashboard.putBoolean("Algae_atAll", atArea && atYaw && atRot);

      SmartDashboard.putNumber("Algae_cmd_vx_mps", commandSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Algae_cmd_vy_mps", commandSpeeds.vyMetersPerSecond);
      SmartDashboard.putNumber("Algae_cmd_omega_rps", commandSpeeds.omegaRadiansPerSecond);
    } else {
      drivebase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
      SmartDashboard.putBoolean("Algae_hasTarget", false);
    }

    SmartDashboard.putNumber("Algae_poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return dontSeeTargetTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME)
        || stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
