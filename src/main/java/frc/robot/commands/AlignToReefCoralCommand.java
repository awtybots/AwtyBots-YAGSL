package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;
import frc.robot.Constants;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToReefCoralCommand extends Command {
        private final SwerveSubsystem swerve;
        private final CoralToReefVisionSubsystem vision;
        private final boolean alignLeft;

        // PID controllers for movement
        // private final ProfiledPIDController distanceController;
        // private final ProfiledPIDController strafeController;
        // private final ProfiledPIDController rotationController;
        private final ProfiledPIDController distanceController; // used for radial distance
        private final ProfiledPIDController rotationController; // used for heading

        private Pose2d targetPose;
        private boolean hasValidTarget = false;

        public AlignToReefCoralCommand(SwerveSubsystem swerve, CoralToReefVisionSubsystem vision, boolean alignLeft) {
                this.swerve = swerve;
                this.vision = vision;
                this.alignLeft = alignLeft;

                addRequirements(swerve, vision);

                // Translation PID
                distanceController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.TRANSLATION_kP,
                                Constants.VisionConstants.Coral.TRANSLATION_kI,
                                Constants.VisionConstants.Coral.TRANSLATION_kD,
                                Constants.VisionConstants.Coral.TRANSLATION_CONSTRAINTS);
                distanceController.setTolerance(0.0254); // ~1 inch

                // Strafe PID
                // strafeController = new ProfiledPIDController(
                // Constants.VisionConstants.Coral.STRAFE_kP,
                // Constants.VisionConstants.Coral.STRAFE_kI,
                // Constants.VisionConstants.Coral.STRAFE_kD,
                // Constants.VisionConstants.Coral.STRAFE_CONSTRAINTS);
                // strafeController.setTolerance(Constants.VisionConstants.Coral.STRAFE_TOLERANCE);

                // Rotation PID
                rotationController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.ROTATION_kP,
                                Constants.VisionConstants.Coral.ROTATION_kI,
                                Constants.VisionConstants.Coral.ROTATION_kD,
                                Constants.VisionConstants.Coral.ROTATION_CONSTRAINTS);
                rotationController.setTolerance(Math.toRadians(1));
                rotationController.enableContinuousInput(-Math.PI, Math.PI);
        }

        @Override
        public void initialize() {
                vision.updateOdometryWithVision();

                // 1) Retrieve your “best reef pose”
                targetPose = vision.getBestReefPos(alignLeft);

                // 2) If invalid, stop
                if (targetPose == null || targetPose.equals(new Pose2d())) {
                        System.out.println("[AlignToReefCoralCommand] No valid scoring pose found. Stopping.");
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }
                hasValidTarget = true;

                // 3) Reset controllers
                Pose2d currentPose = swerve.getPose();
                double distance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                distanceController.reset(distance); // zero velocity, or distanceController.reset(distance, 0.0);

                double currentHeading = currentPose.getRotation().getRadians();
                double currentRotVel = 0.0; // or get from swerve if you track angular velocity
                rotationController.reset(currentHeading, currentRotVel);

                System.out.println("[AlignToReefCoralCommand] STARTED, targetPose=" + targetPose);
        }

        @Override
        public void execute() {
                // 1) If we don't have a valid target from initialize(), do nothing
                if (!hasValidTarget) {
                        System.out.println("No valid target, exiting execute");
                        return;
                }

                // 2) Grab current pose and compute the distance + heading error to target
                Pose2d currentPose = swerve.getPose();

                double targetDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                double distanceError = targetDistance; // We want to get to 0.0m away
                double rotationError = currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians();

                // 3) Calculate PID outputs
                double distanceOutput = distanceController.calculate(distanceError, 0.0);
                double rotationOutput = rotationController.calculate(rotationError, 0.0);

                // 4) If we’re close enough, zero the output (prevents “dancing”)
                if (distanceController.atGoal()) {
                        distanceOutput = 0.0;
                }
                if (rotationController.atGoal()) {
                        rotationOutput = 0.0;
                }

                // 5) Convert distanceOutput into X/Y speed by finding the direction from
                // current to target
                double dx = targetPose.getX() - currentPose.getX();
                double dy = targetPose.getY() - currentPose.getY();
                Rotation2d direction = new Rotation2d(dx, dy);

                double driveX = distanceOutput * direction.getCos();
                double driveY = distanceOutput * direction.getSin();

                // 6) Clamp final outputs to max speeds
                driveX = MathUtil.clamp(
                                driveX,
                                -Constants.VisionConstants.Coral.maxForwardSpeed,
                                Constants.VisionConstants.Coral.maxForwardSpeed);
                driveY = MathUtil.clamp(
                                driveY,
                                -Constants.VisionConstants.Coral.maxStrafeSpeed,
                                Constants.VisionConstants.Coral.maxStrafeSpeed);
                rotationOutput = MathUtil.clamp(
                                rotationOutput,
                                -Constants.VisionConstants.Coral.maxRotationSpeed,
                                Constants.VisionConstants.Coral.maxRotationSpeed);

                // 7) [Optional] Minimum output thresholds to overcome static friction
                double minDriveOutput = 0.4;
                double minRotationOutput = 0.4;

                // For driveX
                if (driveX > 0 && driveX < minDriveOutput) {
                        driveX = minDriveOutput;
                } else if (driveX < 0 && driveX > -minDriveOutput) {
                        driveX = -minDriveOutput;
                }

                // For driveY
                if (driveY > 0 && driveY < minDriveOutput) {
                        driveY = minDriveOutput;
                } else if (driveY < 0 && driveY > -minDriveOutput) {
                        driveY = -minDriveOutput;
                }

                // For rotation
                if (rotationOutput > 0 && rotationOutput < minRotationOutput) {
                        rotationOutput = minRotationOutput;
                } else if (rotationOutput < 0 && rotationOutput > -minRotationOutput) {
                        rotationOutput = -minRotationOutput;
                }

                // 8) Debug logging
                System.out.println("[AlignToReefCoralCommand] EXECUTING");
                System.out.println(" - Current Pose: " + currentPose);
                System.out.println(" - Target Pose:  " + targetPose);
                System.out.println(" - Target Distance: " + targetDistance);
                System.out.println(" - Distance Output: " + distanceOutput);
                System.out.println(" - Rotation Error:  " + rotationError);
                System.out.println(" - Rotation Output: " + rotationOutput);

                SmartDashboard.putNumber("Vision/Target Distance", targetDistance);
                SmartDashboard.putNumber("PID-Vision/Drive X", driveX);
                SmartDashboard.putNumber("PID-Vision/Drive Y", driveY);
                SmartDashboard.putNumber("PID-Vision/Rotation Output", rotationOutput);
                SmartDashboard.putBoolean("Vision/Has Valid Target", hasValidTarget);

                // 9) Command the drive
                swerve.drive(driveX, driveY, rotationOutput);
        }

        @Override
        public void end(boolean interrupted) {
                System.out.println("[AlignToReefCoralCommand] END called. Interrupted: " + interrupted);
                hasValidTarget = false;
                swerve.stop();
        }

        @Override
        public boolean isFinished() {
                // boolean finished = distanceController.atGoal() && strafeController.atGoal()
                // && rotationController.atGoal();
                boolean finished = distanceController.atGoal() && rotationController.atGoal();
                System.out.println("[AlignToReefCoralCommand] isFinished: " + finished);
                SmartDashboard.putBoolean("Vision/Alignment Finished", finished);
                return finished;
        }
}
