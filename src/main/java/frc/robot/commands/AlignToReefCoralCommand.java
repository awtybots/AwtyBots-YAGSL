package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;
import frc.robot.Constants;

import java.util.Optional;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

public class AlignToReefCoralCommand extends Command {
        private final SwerveSubsystem swerve;
        private final CoralToReefVisionSubsystem vision;
        private final boolean alignLeft;

        // PID controllers for movement
        private final ProfiledPIDController translationController;
        private final ProfiledPIDController strafeController;
        private final ProfiledPIDController rotationController;

        private Pose2d targetPose;
        private boolean hasValidTarget = false;

        public AlignToReefCoralCommand(SwerveSubsystem swerve, CoralToReefVisionSubsystem vision, boolean alignLeft) {
                this.swerve = swerve;
                this.vision = vision;
                this.alignLeft = alignLeft;

                addRequirements(swerve, vision);

                // Translation PID
                translationController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.TRANSLATION_kP,
                                Constants.VisionConstants.Coral.TRANSLATION_kI,
                                Constants.VisionConstants.Coral.TRANSLATION_kD,
                                Constants.VisionConstants.Coral.TRANSLATION_CONSTRAINTS);
                translationController.setTolerance(0.0254); // ~1 inch

                // Strafe PID
                strafeController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.STRAFE_kP,
                                Constants.VisionConstants.Coral.STRAFE_kI,
                                Constants.VisionConstants.Coral.STRAFE_kD,
                                Constants.VisionConstants.Coral.STRAFE_CONSTRAINTS);
                strafeController.setTolerance(Constants.VisionConstants.Coral.STRAFE_TOLERANCE);

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
                Pose2d currentPose = swerve.getPose();

                // Option 1: If you want to use fixed coordinates when a tag is detected…
                Optional<Integer> detectedTag = vision.getDetectedTagID();
                if (detectedTag.isPresent()) {
                        // Look up the fixed field pose from constants using the detected tag.
                        targetPose = Constants.VisionConstants.Coral.getBestReefPose(detectedTag.get(), alignLeft);
                        System.out.println("[AlignToReefCoralCommand] Detected tag ID: " + detectedTag.get());
                } else {
                        // Option 2: Otherwise, use the estimated field pose.
                        Optional<Pose2d> estimatedPoseOpt = vision.getTargetPose();
                        if (estimatedPoseOpt.isPresent()) {
                                targetPose = estimatedPoseOpt.get();
                        }
                }

                // If targetPose is still null or default, abort.
                if (targetPose == null || targetPose.equals(new Pose2d())) {
                        System.out.println("[AlignToReefCoralCommand] No valid scoring pose found. Stopping.");
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }

                hasValidTarget = true;
                System.out.println("[AlignToReefCoralCommand] STARTED");
                System.out.println(" - Target Pose: " + targetPose);
        }

        @Override
        public void execute() {
                if (!hasValidTarget) {
                        System.out.println("No valid target, exiting execute");
                        return;
                }

                // Use the estimated field pose as the target.
                Optional<Pose2d> estimatedPoseOpt = vision.getTargetPose();
                if (estimatedPoseOpt.isEmpty()) {
                        System.out.println(
                                        "[AlignToReefCoralCommand] No valid scoring pose found during execution. Stopping.");
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }
                targetPose = estimatedPoseOpt.get();

                Pose2d currentPose = swerve.getPose();
                double targetDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                double lateralOffset = currentPose.getTranslation().getY() - targetPose.getTranslation().getY();
                double rotationError = currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians();

                double forwardSpeed = translationController.calculate(targetDistance, 0);
                double strafeSpeed = strafeController.calculate(lateralOffset, 0);
                double rotationSpeed = rotationController.calculate(rotationError, 0);

                if (translationController.atGoal())
                        forwardSpeed = 0;
                if (strafeController.atGoal())
                        strafeSpeed = 0;
                if (rotationController.atGoal())
                        rotationSpeed = 0;

                forwardSpeed = MathUtil.clamp(forwardSpeed,
                                -Constants.VisionConstants.Coral.maxForwardSpeed,
                                Constants.VisionConstants.Coral.maxForwardSpeed);
                strafeSpeed = MathUtil.clamp(strafeSpeed,
                                -Constants.VisionConstants.Coral.maxStrafeSpeed,
                                Constants.VisionConstants.Coral.maxStrafeSpeed);
                rotationSpeed = MathUtil.clamp(rotationSpeed,
                                -Constants.VisionConstants.Coral.maxRotationSpeed,
                                Constants.VisionConstants.Coral.maxRotationSpeed);

                System.out.println("[AlignToReefCoralCommand] EXECUTING");
                System.out.println(" - Current Pose: " + currentPose);
                System.out.println(" - Target Pose: " + targetPose);
                System.out.println(" - Target Distance: " + targetDistance);
                System.out.println(" - Lateral Offset: " + lateralOffset);
                System.out.println(" - Forward Speed: " + forwardSpeed);
                System.out.println(" - Strafe Speed: " + strafeSpeed);
                System.out.println(" - Rotation Speed: " + rotationSpeed);

                SmartDashboard.putNumber("Vision/Target Distance", targetDistance);
                SmartDashboard.putNumber("Vision/Lateral Offset", lateralOffset);
                SmartDashboard.putNumber("PID-Vision/Forward Speed", forwardSpeed);
                SmartDashboard.putNumber("PID-Vision/Strafe Speed", strafeSpeed);
                SmartDashboard.putNumber("PID-Vision/Rotation Speed", rotationSpeed);
                SmartDashboard.putBoolean("Vision/Has Valid Target", hasValidTarget);

                swerve.drive(forwardSpeed, strafeSpeed, rotationSpeed);
        }

        @Override
        public void end(boolean interrupted) {
                System.out.println("[AlignToReefCoralCommand] END called. Interrupted: " + interrupted);
                hasValidTarget = false;
                swerve.stop();
        }

        @Override
        public boolean isFinished() {
                boolean finished = translationController.atGoal() && strafeController.atGoal()
                                && rotationController.atGoal();
                System.out.println("[AlignToReefCoralCommand] isFinished: " + finished);
                SmartDashboard.putBoolean("Vision/Alignment Finished", finished);
                return finished;
        }
}
