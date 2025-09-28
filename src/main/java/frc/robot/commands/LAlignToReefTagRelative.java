// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.ScoreSafetyManager;

public class LAlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private ProfiledPIDController rotControllerProfiled;
  // private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = -1;
  private double lastPoseValidatedTimestamp = -1;
  private int dashboardLoopCounter = Math.max(0, Constants.DASHBOARD_UPDATE_PERIOD_CYCLES - 1);
  private boolean completionReported = false;
  private Pose2d lastPoseWhileVision = null;
  private double lastVisionPoseTimestamp = -1;
  private boolean atPoseWithVisionLatched = false;

  private boolean shouldUpdateDashboard() {
    if (!Constants.LIMIT_DASHBOARD_PERIODIC_UPDATES || Constants.DASHBOARD_UPDATE_PERIOD_CYCLES <= 1) {
      return true;
    }
    dashboardLoopCounter++;
    if (dashboardLoopCounter >= Constants.DASHBOARD_UPDATE_PERIOD_CYCLES) {
      dashboardLoopCounter = 0;
      return true;
    }
    return false;
  }

  public LAlignToReefTagRelative(SwerveSubsystem drivebase) {
    // Forward/back: bump P up if the robot creeps in too slowly, drop it if it
    // shoots past the tag.
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);
    // Strafe: raise this P when the chassis stays offset left/right of the reef,
    // lower if it oscillates.
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);
    // Yaw: tune this when the robot finishes facing left/right instead of square to
    // the reef.
    rotController = new PIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0);
    // Rotation
    // rotControllerProfiled = new
    // ProfiledPIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0,
    // new TrapezoidProfile.Constraints(6.28, 3.14));
    // Rotation using holonic drive controller

    // var controller = new HolonomicDriveController(
    // new PIDController(Constants.X_REEF_ALIGNMENT_P, 0, 0), new
    // PIDController(Constants.Y_REEF_ALIGNMENT_P, 0, 0),
    // new ProfiledPIDController(Constants.ROT_REEF_ALIGNMENT_P, 0, 0,
    // new TrapezoidProfile.Constraints(6.28, 3.14)));
    // // Here, our rotation profile constraints were a max velocity
    // // of 1 rotation per second and a max acceleration of 180 degrees
    // // per second squared.
    // this.isRightScore = isRightScore;
    this.drivebase = drivebase;
    addRequirements(drivebase);
  }

  @Override
  public void initialize() {
    this.stopTimer = new Timer();
    this.stopTimer.start();
    this.dontSeeTagTimer = new Timer();
    this.dontSeeTagTimer.start();
    lastPoseValidatedTimestamp = -1;
    completionReported = false;
    lastPoseWhileVision = null;
    lastVisionPoseTimestamp = -1;
    atPoseWithVisionLatched = false;

    SmartDashboard.putBoolean("AutoAlignLeftComplete", false);

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    // If the robot stops short or bumps the reef, shift this X setpoint (positive
    // pulls closer).
    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    // Move this Y setpoint toward zero when the chassis finishes too far left of
    // the pole.
    yController.setSetpoint(Constants.Y_L_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    // Latch the first tag we see so the robot does not jump between IDs mid-align.
    tagID = LimelightHelpers.getFiducialID("limelight-right");
  }

  @Override
  // public void execute() {
  // if (LimelightHelpers.getTV("limelight-right") &&
  // LimelightHelpers.getFiducialID("limelight-right") == tagID) {
  // this.dontSeeTagTimer.reset();

  // double[] postions =
  // LimelightHelpers.getBotPose_TargetSpace("limelight-right");
  // SmartDashboard.putNumber("x", postions[2]);

  // double xSpeed = -xController.calculate(postions[2]);
  // SmartDashboard.putNumber("xspeed", xSpeed);
  // double ySpeed = yController.calculate(postions[0]);
  // double rotValue = rotController.calculate(postions[4]);

  // drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

  // if (!rotController.atSetpoint() ||
  // !yController.atSetpoint() ||
  // !xController.atSetpoint()) {
  // stopTimer.reset();
  // }
  // } else {
  // drivebase.drive(new Translation2d(), 0, false);
  // }

  // SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  // }
  public void execute() {
    boolean updateDashboard = shouldUpdateDashboard();
    if (LimelightHelpers.getTV("limelight-right")
        && LimelightHelpers.getFiducialID("limelight-right") == tagID) {
      this.dontSeeTagTimer.reset();

      double[] postions = LimelightHelpers.getBotPose_TargetSpace("limelight-right");

      double xSpeed = -xController.calculate(postions[2]);
      double ySpeed = yController.calculate(postions[0]);
      double rotValue = rotController.calculate(postions[4]);

      boolean atPose = rotController.atSetpoint() && yController.atSetpoint() && xController.atSetpoint();
      double now = Timer.getFPGATimestamp();
      if (atPose) {
        atPoseWithVisionLatched = true;
        lastPoseWhileVision = drivebase.getPose();
        lastVisionPoseTimestamp = now;
        drivebase.stop();
        if (stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME)) {
          recordPoseValidation();
        }
      } else {
        stopTimer.reset();
        lastPoseValidatedTimestamp = -1;
        atPoseWithVisionLatched = false;
        lastPoseWhileVision = null;
        lastVisionPoseTimestamp = -1;
        drivebase.drive(
            new Translation2d(
                // If we jump forward before we are centered, increase the Y tolerance gate or
                // lower this 0.03 safety creep.
                yController.getError() < 0.3 ? xSpeed : 0.00,
                ySpeed),
            rotValue,
            false);
      }

      if (updateDashboard) {
        SmartDashboard.putNumber("xspeed", xSpeed);
      }
    } else {
      // drivebase.drive(
      // new Translation2d(),
      // 0,
      // false);
      drivebase.stop();
      if (atPoseWithVisionLatched && lastPoseWhileVision != null && lastVisionPoseTimestamp > 0) {
        double timeSinceVision = Timer.getFPGATimestamp() - lastVisionPoseTimestamp;
        boolean withinGrace = timeSinceVision <= Constants.POSE_LOSS_GRACE_PERIOD;
        boolean poseStillValid = isCurrentPoseCloseToVisionPose();
        if (withinGrace && stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME) && poseStillValid) {
          recordPoseValidation();
        } else if (!withinGrace || !poseStillValid) {
          atPoseWithVisionLatched = false;
          lastPoseWhileVision = null;
          lastVisionPoseTimestamp = -1;
        }
      }
      if (updateDashboard) {
        SmartDashboard.putNumber("xspeed", 0);
      }
    }

    if (updateDashboard) {
      SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
    }
  }

  @Override
  public void end(boolean interrupted) {
      // drivebase.drive(new Translation2d(), 0, false);
    drivebase.stop();
    if (!interrupted) {
      ScoreSafetyManager.activateLockout(drivebase.getPose());
    }
    if (!completionReported) {
      SmartDashboard.putBoolean("AutoAlignLeftComplete", false);
    }
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long
    // as it gets a tag in the camera
    // Extend DONT_SEE_TAG_WAIT_TIME if small camera dropouts end the command too early.
    double now = Timer.getFPGATimestamp();
    boolean poseRecentlyValidated = lastPoseValidatedTimestamp > 0
        && (now - lastPoseValidatedTimestamp) <= Constants.POSE_LOSS_GRACE_PERIOD;
    return poseRecentlyValidated
        || this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME);
  }

  private void recordPoseValidation() {
    lastPoseValidatedTimestamp = Timer.getFPGATimestamp();
    if (!completionReported) {
      DriverStation.reportWarning("Auto align left finished", false);
      System.out.println("Auto align left finished");
      SmartDashboard.putBoolean("AutoAlignLeftComplete", true);
      completionReported = true;
    }
  }

  private boolean isCurrentPoseCloseToVisionPose() {
    if (lastPoseWhileVision == null) {
      return false;
    }
    Pose2d currentPose = drivebase.getPose();
    double translationError =
        currentPose.getTranslation().getDistance(lastPoseWhileVision.getTranslation());
    double headingError = Math.abs(
        currentPose.getRotation().minus(lastPoseWhileVision.getRotation()).getDegrees());
    return translationError <= Constants.POSE_ODOMETRY_TOLERANCE_METERS
        && headingError <= Constants.POSE_ODOMETRY_TOLERANCE_DEGREES;
  }
}
