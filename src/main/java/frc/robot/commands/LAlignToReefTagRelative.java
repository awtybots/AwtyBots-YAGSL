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

public class LAlignToReefTagRelative extends Command {
  // Controllers for HolonomicDriveController
  private final PIDController xController; // Tag-space Z axis mapped to X in controller frame
  private final PIDController yController; // Tag-space X axis mapped to Y in controller frame
  private final ProfiledPIDController thetaController;
  private final HolonomicDriveController holonomic;

  private Timer dontSeeTagTimer, stopTimer;
  private final SwerveSubsystem drivebase;
  private int tagID = -1;

  public LAlignToReefTagRelative(SwerveSubsystem drivebase) {
    // PID gains tune how aggressively we correct tag-space Z (forward/back) error.
    // increase X_REEF_ALIGNMENT_P for faster approach or tweak D term to reduce overshoot.
    this.xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0.0);
    // Controls tag-space X (left/right) correction while holding yaw constant.
    this.yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0.0);

    // Profiled PID governs yaw error toward the reef; adjust ROT_REEF_ALIGNMENT_P for rotation response.
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

    this.tagID = -1; // Lazily latch target tag when we first see it

    // Tolerances for setpoint checks (HolonomicDriveController handles control)
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);
    thetaController.setTolerance(Math.toRadians(Constants.ROT_TOLERANCE_REEF_ALIGNMENT));

  }

  @Override
  public void execute() {
    final String llNameR = "limelight-right";
    // Only align when the Limelight has a valid tag and it matches the one we latched
    boolean hasTarget = LimelightHelpers.getTV(llNameR);
    if (hasTarget) {
      int seenTag = (int) Math.round(LimelightHelpers.getFiducialID(llNameR));
      if (seenTag <= 0) {
        hasTarget = false;
      } else {
        if (tagID < 0) {
          // First valid detection since init – remember which tag we want to finish on
          tagID = seenTag;
        }
        hasTarget = (seenTag == tagID);
      }
    }

    if (hasTarget) {
      this.dontSeeTagTimer.reset();

      // Target-space pose of robot (translation: [0]=X, [2]=Z, rotation yaw: [4])
      double[] positions = LimelightHelpers.getBotPose_TargetSpace(llNameR);

      // Latency compensation: predict where the robot is NOW in target-space based on
      // robot-relative velocity and Limelight latency.
      double llLatencySec =
          (LimelightHelpers.getLatency_Pipeline(llNameR) + LimelightHelpers.getLatency_Capture(llNameR)) / 1000.0;

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
      double predZ = positions[2] - vTargetZ * llLatencySec; // tag-space forward/back
      double predX = positions[0] - vTargetX * llLatencySec; // tag-space left/right
      double predYawDeg = positions[4] + Math.toDegrees(omega * llLatencySec);

      // Map tag-space Z -> controller X, tag-space X -> controller Y
      Pose2d currentTagRelativePose = new Pose2d(
          -predZ,
          predX,
          Rotation2d.fromDegrees(predYawDeg));

      Pose2d goalTagRelativePose = new Pose2d(
          -Constants.X_SETPOINT_REEF_ALIGNMENT,
          Constants.Y_L_SETPOINT_REEF_ALIGNMENT,
          Rotation2d.fromDegrees(Constants.ROT_SETPOINT_REEF_ALIGNMENT));

      // Calculate robot-relative speeds (tag-relative control frame)
      ChassisSpeeds outputSpeeds = holonomic.calculate(
          currentTagRelativePose,
          goalTagRelativePose,
          0.0,
          Rotation2d.fromDegrees(Constants.ROT_SETPOINT_REEF_ALIGNMENT));

      // Drive using robot-relative speeds (equivalent to fieldRelative=false)
      drivebase.drive(outputSpeeds);

      // Setpoint adherence logic (same behavior as before)
      double xErr = predZ - Constants.X_SETPOINT_REEF_ALIGNMENT;
      double yErr = Constants.Y_L_SETPOINT_REEF_ALIGNMENT - predX;
      double rotErrDeg = Constants.ROT_SETPOINT_REEF_ALIGNMENT - predYawDeg;

      boolean atX = Math.abs(xErr) <= Constants.X_TOLERANCE_REEF_ALIGNMENT;
      boolean atY = Math.abs(yErr) <= Constants.Y_TOLERANCE_REEF_ALIGNMENT;
      boolean atRot = Math.abs(rotErrDeg) <= Constants.ROT_TOLERANCE_REEF_ALIGNMENT;

      if (!(atX && atY && atRot)) {
        stopTimer.reset();
      }

      // SmartDashboard logging for tuning
      // SmartDashboard.putNumber("Align_Z_meas", positions[2]);
      // SmartDashboard.putNumber("Align_Z_pred", predZ);
      // SmartDashboard.putNumber("Align_X_meas", positions[0]);
      // SmartDashboard.putNumber("Align_X_pred", predX);
      // SmartDashboard.putNumber("Align_Yaw_meas_deg", positions[4]);
      // SmartDashboard.putNumber("Align_Yaw_pred_deg", predYawDeg);

      // SmartDashboard.putNumber("Align_err_X(m)", xErr);
      // SmartDashboard.putNumber("Align_err_Y(m)", yErr);
      // SmartDashboard.putNumber("Align_err_Yaw(deg)", rotErrDeg);

      // SmartDashboard.putBoolean("Align_atX", atX);
      // SmartDashboard.putBoolean("Align_atY", atY);
      // SmartDashboard.putBoolean("Align_atRot", atRot);
      // SmartDashboard.putBoolean("Align_atAll", atX && atY && atRot);

      // SmartDashboard.putNumber("Align_cmd_vx(mps)", outputSpeeds.vxMetersPerSecond);
      // SmartDashboard.putNumber("Align_cmd_vy(mps)", outputSpeeds.vyMetersPerSecond);
      // SmartDashboard.putNumber("Align_cmd_omega(rps)", outputSpeeds.omegaRadiansPerSecond);
    } else {
      // No valid tag - stop
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
    // Requires the robot to stay in the correct position for 0.3 seconds, as long
    // as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME)
        || stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
