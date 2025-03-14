package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

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
                Pose2d targetPose = vision.getBestReefPos(alignLeft);

                if (targetPose == null) {
                        System.out.println("[AlignToReefCoralCommand] No valid scoring pose found. Stopping.");
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }

                hasValidTarget = true;
                System.out.println("[AlignToReefCoralCommand] STARTED");
        }

        @Override
        public void execute() {
                if (!hasValidTarget) {
                        return;
                }

                Pose2d currentPose = swerve.getPose();
                Pose2d targetPose = vision.getBestReefPos(alignLeft);
                if (targetPose == null) {
                        System.out.println("[AlignToReefCoralCommand] No valid scoring pose found. Stopping.");
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }

                // Compute translation and rotation errors
                double targetDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                double lateralOffset = currentPose.getTranslation().getY() - targetPose.getTranslation().getY();
                double rotationError = currentPose.getRotation().getRadians() - targetPose.getRotation().getRadians();

                // Compute PID outputs
                double forwardSpeed = translationController.calculate(targetDistance, 0);
                double strafeSpeed = strafeController.calculate(lateralOffset, 0);
                double rotationSpeed = rotationController.calculate(rotationError, 0);

                // Stop movement if within tolerance
                if (translationController.atGoal())
                        forwardSpeed = 0;
                if (strafeController.atGoal())
                        strafeSpeed = 0;
                if (rotationController.atGoal())
                        rotationSpeed = 0;

                // Clamp speeds
                forwardSpeed = MathUtil.clamp(forwardSpeed,
                                -Constants.VisionConstants.Coral.maxForwardSpeed,
                                Constants.VisionConstants.Coral.maxForwardSpeed);
                strafeSpeed = MathUtil.clamp(strafeSpeed,
                                -Constants.VisionConstants.Coral.maxStrafeSpeed,
                                Constants.VisionConstants.Coral.maxStrafeSpeed);
                rotationSpeed = MathUtil.clamp(rotationSpeed,
                                -Constants.VisionConstants.Coral.maxRotationSpeed,
                                Constants.VisionConstants.Coral.maxRotationSpeed);

                // Drive the robot
                swerve.drive(forwardSpeed, strafeSpeed, rotationSpeed);
        }

        @Override
        public void end(boolean interrupted) {
                hasValidTarget = false;
                swerve.stop();
        }

        @Override
        public boolean isFinished() {
                return translationController.atGoal() && strafeController.atGoal() && rotationController.atGoal();
        }
}
