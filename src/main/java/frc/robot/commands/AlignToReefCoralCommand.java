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
                distanceController.setTolerance(Constants.VisionConstants.Coral.TRANSLATION_TOLERANCE); // ~1 inch

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

                swerve.setFieldOriented(true);

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
                        System.out.println("[AlignToReefCoralCommand] No valid target, exiting execute.");
                        return;
                }

                // 2) Grab current pose and compute the distance + heading error to target
                Pose2d currentPose = swerve.getPose();
                double currentX = currentPose.getX();
                double currentY = currentPose.getY();
                double currentRotation = currentPose.getRotation().getRadians();
                double targetX = targetPose.getX();
                double targetY = targetPose.getY();
                double targetRotation = targetPose.getRotation().getRadians();

                double targetDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                double distanceError = targetDistance; // We want to get to 0.0m away
                double rotationError = currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians();

                // Normalize rotation error to [-π, π] range to avoid excessive spinning
                rotationError = Math.atan2(Math.sin(rotationError), Math.cos(rotationError));

                // Debug logs before PID calculation
                System.out.println("[AlignToReefCoralCommand] --- ERROR VALUES ---");
                System.out.println("Current X: " + currentX + " | Target X: " + targetX + " | X Error: "
                                + (targetX - currentX));
                System.out.println("Current Y: " + currentY + " | Target Y: " + targetY + " | Y Error: "
                                + (targetY - currentY));
                System.out.println("Target Distance: " + targetDistance + " | Distance Error: " + distanceError);
                System.out.println("Rotation Error (radians): " + rotationError + " | Rotation Error (degrees): "
                                + Math.toDegrees(rotationError));

                // 3) Calculate PID outputs
                double distanceOutput = distanceController.calculate(distanceError, 0.0);
                double rotationOutput = rotationController.calculate(rotationError, 0.0);

                // Log raw PID outputs
                System.out.println("[AlignToReefCoralCommand] --- RAW PID OUTPUTS ---");
                System.out.println("Distance Output: " + distanceOutput);
                System.out.println("Rotation Output: " + rotationOutput);

                // 4) Zero outputs if we're within tolerance
                boolean distanceAtGoal = distanceController.atGoal();
                boolean rotationAtGoal = rotationController.atGoal();

                if (distanceAtGoal) {
                        System.out.println("[AlignToReefCoralCommand] Distance PID at goal. Zeroing distance output.");
                        distanceOutput = 0.0;
                }
                if (rotationAtGoal) {
                        System.out.println("[AlignToReefCoralCommand] Rotation PID at goal. Zeroing rotation output.");
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

                // 7) Apply minimum output thresholds to overcome static friction
                double minDriveOutput = 0.1;
                double minRotationOutput = 0.4;

                driveX = applyMinimumThreshold(driveX, minDriveOutput);
                driveY = applyMinimumThreshold(driveY, minDriveOutput);
                rotationOutput = applyMinimumThreshold(rotationOutput, minRotationOutput);

                // 8) Debug logging
                System.out.println("[AlignToReefCoralCommand] --- FINAL OUTPUTS ---");
                System.out.println("Drive X (Adjusted): " + driveX);
                System.out.println("Drive Y (Adjusted): " + driveY);
                System.out.println("Rotation Output (Adjusted): " + rotationOutput);

                SmartDashboard.putNumber("Vision/Target Distance", targetDistance);
                SmartDashboard.putNumber("PID-Vision/Drive X", driveX);
                SmartDashboard.putNumber("PID-Vision/Drive Y", driveY);
                SmartDashboard.putNumber("PID-Vision/Rotation Output", rotationOutput);
                SmartDashboard.putBoolean("Vision/Has Valid Target", hasValidTarget);
                rotationOutput = -rotationOutput;
                // 9) Command the drive
                swerve.drive(driveX, driveY, rotationOutput);
        }

        // Helper function to apply minimum output thresholds
        private double applyMinimumThreshold(double value, double threshold) {
                if (value > 0 && value < threshold) {
                        return threshold;
                } else if (value < 0 && value > -threshold) {
                        return -threshold;
                }
                return value;
        }

        @Override
        public void end(boolean interrupted) {
                System.out.println("[AlignToReefCoralCommand] END called. Interrupted: " + interrupted);
                hasValidTarget = false;
                swerve.setFieldOriented(true);
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
