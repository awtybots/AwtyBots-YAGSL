// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.subsystems.SwerveSubsystem;

public class LAlignToReefTagRelative extends Command {
  private PIDController xController, yController, rotController;
  private ProfiledPIDController rotControllerProfiled;
  // private boolean isRightScore;
  private Timer dontSeeTagTimer, stopTimer;
  private SwerveSubsystem drivebase;
  private double tagID = -1;

  public LAlignToReefTagRelative(SwerveSubsystem drivebase) {
    xController = new PIDController(Constants.X_REEF_ALIGNMENT_P, 0.0, 0);
    // Vertical movement
    yController = new PIDController(Constants.Y_REEF_ALIGNMENT_P, 0.0, 0);
    // Horitontal movement
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

    rotController.setSetpoint(Constants.ROT_SETPOINT_REEF_ALIGNMENT);
    rotController.setTolerance(Constants.ROT_TOLERANCE_REEF_ALIGNMENT);

    xController.setSetpoint(Constants.X_SETPOINT_REEF_ALIGNMENT);
    xController.setTolerance(Constants.X_TOLERANCE_REEF_ALIGNMENT);

    yController.setSetpoint(Constants.Y_L_SETPOINT_REEF_ALIGNMENT);
    yController.setTolerance(Constants.Y_TOLERANCE_REEF_ALIGNMENT);

    tagID = LimelightHelpers.getFiducialID("limelight-right");
  }

  @Override
  public void execute() {
    final String llName = "limelight-right";
    if (LimelightHelpers.getTV(llName) && LimelightHelpers.getFiducialID(llName) == tagID) {
      this.dontSeeTagTimer.reset();

      // Target-space pose of robot (translation: [0]=X, [2]=Z, rotation yaw: [4])
      double[] positions = LimelightHelpers.getBotPose_TargetSpace(llName);
      SmartDashboard.putNumber("x", positions[2]);
      // Latency compensation: predict where the robot is NOW in target-space
      // based on robot-relative velocity and limelight latency.
      double llLatencySec = (LimelightHelpers.getLatency_Pipeline(llName) + LimelightHelpers.getLatency_Capture(llName))
          / 1000.0;

      // Robot relative chassis speeds
      var speeds = drivebase.getRobotRelativeSpeeds();
      double vx = speeds.vxMetersPerSecond; // +X forward
      double vy = speeds.vyMetersPerSecond; // +Y left
      double omega = speeds.omegaRadiansPerSecond; // CCW +

      // Robot yaw relative to target-space (degrees in LL array -> radians)
      double rYawRad = Math.toRadians(positions[4]);

      // Transform robot-frame velocity into target-space components (Z forward/back,
      // X left/right)
      double vTargetZ = vx * Math.cos(rYawRad) - vy * Math.sin(rYawRad);
      double vTargetX = vx * Math.sin(rYawRad) + vy * Math.cos(rYawRad);

      // Predict current target-space pose by subtracting motion during latency
      double predZ = positions[2] - vTargetZ * llLatencySec;
      double predX = positions[0] - vTargetX * llLatencySec;
      double predYawDeg = positions[4] + Math.toDegrees(omega * llLatencySec);

      SmartDashboard.putNumber("Align_Z_meas", positions[2]);
      SmartDashboard.putNumber("Align_Z_pred", predZ);

      double xSpeed = -xController.calculate(predZ);
      SmartDashboard.putNumber("xspeed", xSpeed);
      double ySpeed = yController.calculate(predX);
      double rotValue = rotController.calculate(predYawDeg);

      drivebase.drive(new Translation2d(xSpeed, ySpeed), rotValue, false);

      if (!rotController.atSetpoint() ||
          !yController.atSetpoint() ||
          !xController.atSetpoint()) {
        stopTimer.reset();
      }
    } else {
      drivebase.drive(new Translation2d(), 0, false);
    }

    SmartDashboard.putNumber("poseValidTimer", stopTimer.get());
  }

  @Override
  public void end(boolean interrupted) {
    drivebase.drive(new Translation2d(), 0, false);
  }

  @Override
  public boolean isFinished() {
    // Requires the robot to stay in the correct position for 0.3 seconds, as long
    // as it gets a tag in the camera
    return this.dontSeeTagTimer.hasElapsed(Constants.DONT_SEE_TAG_WAIT_TIME) ||
        stopTimer.hasElapsed(Constants.POSE_VALIDATION_TIME);
  }
}
