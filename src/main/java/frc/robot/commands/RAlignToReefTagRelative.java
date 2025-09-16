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
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class RAlignToReefTagRelative extends Command {
  private final PIDController xController;
  private final PIDController yController;
  private final ProfiledPIDController thetaController;
  private final HolonomicDriveController holonomic;

  private Timer dontSeeTagTimer, stopTimer;
  private final SwerveSubsystem drivebase;
  private int tagID = -1;

  public RAlignToReefTagRelative(SwerveSubsystem drivebase) {
    // PID gain scales forward/back correction in tag-space Z; tweak constants when approach speed feels off.
    this.xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0.01);
    // Governs lateral (tag-space X) correction to stay centered on the reef.
    this.yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0.01);

    // Profiled yaw controller – adjust ROT_REEF_ALIGNMENT_P for rotational responsiveness.
    this.thetaController = new ProfiledPIDController(
        Constants.ROT_REEF_ALIGNMENT_P, 0.0, 0.0,
        new TrapezoidProfile.Constraints(6.28, 3.14));
    this.thetaController.enableContinuousInput(-Math.PI, Math.PI);

    this.holonomic = new HolonomicDriveController(xController, yController, thetaController);

    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();

    this.tagID = -1;

    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);
    thetaController.setTolerance(Math.toRadians(Constants.ROT_TOLERANCE_REEF_ALIGNMENT));

  }

  @Override
  public void execute() {
    final String llName = "limelight-left";
    // Only drive when the Limelight reports a valid target and it matches the latched tag ID
    boolean hasTarget = LimelightHelpers.getTV(llName);
    if (hasTarget) {
      int seenTag = (int) Math.round(LimelightHelpers.getFiducialID(llName));
      if (seenTag <= 0) {
        hasTarget = false;
      } else {
        if (tagID < 0) {
          // First valid detection after initialize – remember which tag we will stick with
          tagID = seenTag;
        }
        hasTarget = (seenTag == tagID);
      }
    }

    if (hasTarget) {
      this.dontSeeTagTimer.reset();

      // Target-space pose of robot (translation: [0]=X, [2]=Z, rotation yaw: [4])
      double[] positions = LimelightHelpers.getBotPose_TargetSpace(llName);

      // Latency compensation: use current robot velocity to predict present-time tag pose
      double llLatencySec = (LimelightHelpers.getLatency_Pipeline(llName)
          + LimelightHelpers.getLatency_Capture(llName)) / 1000.0;

      // Robot relative chassis speeds
      var speeds = drivebase.getRobotRelativeSpeeds();
      double vx = speeds.vxMetersPerSecond; // +X forward
      double vy = speeds.vyMetersPerSecond; // +Y left
      double omega = speeds.omegaRadiansPerSecond; // CCW +

      // Robot yaw relative to target-space (degrees in LL array -> radians)
      double rYawRad = Math.toRadians(positions[4]);

      // Transform robot-frame velocity into target-space components (Z forward/back, X left/right)
      double vTargetZ = vx * Math.cos(rYawRad) - vy * Math.sin(rYawRad);
      double vTargetX = vx * Math.sin(rYawRad) + vy * Math.cos(rYawRad);

      // Predict current target-space pose by subtracting motion during latency
      double predZ = positions[2] - vTargetZ * llLatencySec;
      double predX = positions[0] - vTargetX * llLatencySec;
      double predYawDeg = positions[4] + Math.toDegrees(omega * llLatencySec);

      // Build current and goal poses in tag-relative frame (Z->X, X->Y)
      Pose2d currentTagRelativePose = new Pose2d(
          predZ,
          predX,
          Rotation2d.fromDegrees(predYawDeg));

      Pose2d goalTagRelativePose = new Pose2d(
          Constants.X_SETPOINT_REEF_ALIGNMENT,
          Constants.Y_R_SETPOINT_REEF_ALIGNMENT,
          Rotation2d.fromDegrees(Constants.ROT_SETPOINT_REEF_ALIGNMENT));

      // Calculate robot-relative speeds (tag-relative frame)
      ChassisSpeeds outputSpeeds = holonomic.calculate(
          currentTagRelativePose,
          goalTagRelativePose,
          0.0,
          Rotation2d.fromDegrees(Constants.ROT_SETPOINT_REEF_ALIGNMENT));

      // Drive robot-relative/tag-relative
      drivebase.drive(outputSpeeds);

      // Setpoint checks for stop timer
      double xErr = Constants.X_SETPOINT_REEF_ALIGNMENT - predZ;
      double yErr = Constants.Y_R_SETPOINT_REEF_ALIGNMENT - predX;
      double rotErrDeg = Constants.ROT_SETPOINT_REEF_ALIGNMENT - predYawDeg;

      boolean atX = Math.abs(xErr) <= Constants.X_TOLERANCE_REEF_ALIGNMENT;
      boolean atY = Math.abs(yErr) <= Constants.Y_TOLERANCE_REEF_ALIGNMENT;
      boolean atRot = Math.abs(rotErrDeg) <= Constants.ROT_TOLERANCE_REEF_ALIGNMENT;

      if (!(atX && atY && atRot)) {
        stopTimer.reset();
      }

      // Logging for tuning
      SmartDashboard.putNumber("Align_Z_meas", positions[2]);
      SmartDashboard.putNumber("Align_Z_pred", predZ);
      SmartDashboard.putNumber("Align_X_meas", positions[0]);
      SmartDashboard.putNumber("Align_X_pred", predX);
      SmartDashboard.putNumber("Align_Yaw_meas_deg", positions[4]);
      SmartDashboard.putNumber("Align_Yaw_pred_deg", predYawDeg);

      SmartDashboard.putNumber("Align_err_X(m)", xErr);
      SmartDashboard.putNumber("Align_err_Y(m)", yErr);
      SmartDashboard.putNumber("Align_err_Yaw(deg)", rotErrDeg);

      SmartDashboard.putBoolean("Align_atX", atX);
      SmartDashboard.putBoolean("Align_atY", atY);
      SmartDashboard.putBoolean("Align_atRot", atRot);
      SmartDashboard.putBoolean("Align_atAll", atX && atY && atRot);

      SmartDashboard.putNumber("Align_cmd_vx(mps)", outputSpeeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Align_cmd_vy(mps)", outputSpeeds.vyMetersPerSecond);
      SmartDashboard.putNumber("Align_cmd_omega(rps)", outputSpeeds.omegaRadiansPerSecond);
    } else {
      drivebase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  @Override
  public boolean isFinished() {
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME)
        || stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
