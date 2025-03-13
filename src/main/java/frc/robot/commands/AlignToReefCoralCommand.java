package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

public class AlignToReefCoralCommand extends Command {
        private final SwerveSubsystem swerve;
        private final CoralToReefVisionSubsystem vision;
        private final boolean alignLeft;

        private final ProfiledPIDController translationController;
        private final ProfiledPIDController strafeController;
        private final ProfiledPIDController rotationController;

        private boolean hasValidTarget = false;

        public AlignToReefCoralCommand(SwerveSubsystem swerve, CoralToReefVisionSubsystem vision, boolean alignLeft) {
                this.swerve = swerve;
                this.vision = vision;
                this.alignLeft = alignLeft;

                addRequirements(swerve, vision);

                // PID Controllers
                translationController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.TRANSLATION_kP,
                                Constants.VisionConstants.Coral.TRANSLATION_kI,
                                Constants.VisionConstants.Coral.TRANSLATION_kD,
                                Constants.VisionConstants.Coral.TRANSLATION_CONSTRAINTS);

                strafeController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.STRAFE_kP,
                                Constants.VisionConstants.Coral.STRAFE_kI,
                                Constants.VisionConstants.Coral.STRAFE_kD,
                                Constants.VisionConstants.Coral.STRAFE_CONSTRAINTS);

                rotationController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.ROTATION_kP,
                                Constants.VisionConstants.Coral.ROTATION_kI,
                                Constants.VisionConstants.Coral.ROTATION_kD,
                                Constants.VisionConstants.Coral.ROTATION_CONSTRAINTS);

                // Set tolerances
                translationController.setTolerance(Constants.VisionConstants.Coral.TRANSLATION_TOLERANCE);
                strafeController.setTolerance(Constants.VisionConstants.Coral.STRAFE_TOLERANCE);
                rotationController.setTolerance(Constants.VisionConstants.Coral.ROTATION_TOLERANCE);
        }

        @Override
        public void initialize() {
                vision.updateOdometryWithVision();

                Optional<Pair<Integer, Pose2d>> fieldPoseOpt = vision.getEstimatedFieldPose();
                System.out.println("[AlignToReefCoralCommand] STARTED");
                System.out.println(" - Aligning to: " + (alignLeft ? "LEFT" : "RIGHT") + " Reef");
                if (fieldPoseOpt.isEmpty()) {
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }

                hasValidTarget = true;
        }

        @Override
        public void execute() {
                Optional<Pair<Integer, Pose2d>> fieldPoseOpt = vision.getEstimatedFieldPose();
                if (fieldPoseOpt.isEmpty()) {
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }

                int detectedAprilTagID = fieldPoseOpt.get().getFirst(); // Extract AprilTag ID
                Pose2d currentPose = vision.getEstimatedFieldPose()
                                .map(Pair::getSecond)
                                .orElse(swerve.getPose());

                // Get the Correct Predefined Scoring Pose Based on Alliance & AprilTag
                // ID**
                Pose2d targetPose = Constants.VisionConstants.Coral.getBestReefPose(detectedAprilTagID, alignLeft);

                // Compute Field-Centric Distance
                Translation2d translationError = targetPose.getTranslation().minus(currentPose.getTranslation());
                double targetDistance = translationError.getNorm();

                // Compute Robot Velocity Projection for Smoothed Movement
                ChassisSpeeds robotVelocity = swerve.getRobotVelocity();
                Translation2d velocityVector = new Translation2d(robotVelocity.vxMetersPerSecond,
                                robotVelocity.vyMetersPerSecond);

                // Prevent division by zero
                double translationErrorNorm = translationError.getNorm();
                double velocityProjection = (translationErrorNorm > 0.001)
                                ? (velocityVector.getX() * translationError.getX()
                                                + velocityVector.getY() * translationError.getY())
                                                / translationErrorNorm
                                : 0.0;

                // PID Control for Translation & Rotation
                double forwardSpeed = translationController.calculate(targetDistance, 0) - velocityProjection;

                double rotationPIDOutput = rotationController.calculate(
                                swerve.getGyroYaw(), targetPose.getRotation().getDegrees()); // 🔄 Use gyro-based
                                                                                             // rotation

                double lateralOffset = currentPose.getTranslation().getY() - targetPose.getTranslation().getY();
                double strafeSpeed = strafeController.calculate(lateralOffset, 0);

                // Apply Stop Conditions When Close Enough
                if (targetDistance < Constants.VisionConstants.Coral.TRANSLATION_TOLERANCE) {
                        forwardSpeed = 0;
                        translationController.reset(0);
                }

                if (Math.abs(lateralOffset) < Constants.VisionConstants.Coral.STRAFE_TOLERANCE) {
                        strafeSpeed = 0;
                        strafeController.reset(0);
                }

                if (Math.abs(swerve.getGyroYaw() - targetPose.getRotation()
                                .getDegrees()) < Constants.VisionConstants.Coral.ROTATION_TOLERANCE) {
                        rotationPIDOutput = 0;
                        rotationController.reset(targetPose.getRotation().getDegrees());
                }

                // Limit Speed to Prevent Overcorrection
                forwardSpeed = MathUtil.clamp(forwardSpeed,
                                -Constants.VisionConstants.Coral.maxForwardSpeed,
                                Constants.VisionConstants.Coral.maxForwardSpeed);
                strafeSpeed = MathUtil.clamp(strafeSpeed,
                                -Constants.VisionConstants.Coral.maxStrafeSpeed,
                                Constants.VisionConstants.Coral.maxStrafeSpeed);
                rotationPIDOutput = MathUtil.clamp(rotationPIDOutput,
                                -Constants.VisionConstants.Coral.maxRotationSpeed,
                                Constants.VisionConstants.Coral.maxRotationSpeed);

                SmartDashboard.putNumber("PID-Vision/10 PID-Forward Speed", forwardSpeed);
                SmartDashboard.putNumber("PID-Vision/11 PID-Strafe Speed", strafeSpeed);
                SmartDashboard.putNumber("PID-Vision/12 PID-Rotation Speed", rotationPIDOutput);

                swerve.drive(forwardSpeed, strafeSpeed, rotationPIDOutput);
        }

        @Override
        public void end(boolean interrupted) {
                hasValidTarget = false;
                swerve.stop();
                System.out.println("AlignToReefCoralCommand ended. Interrupted: " + interrupted);
        }

        @Override
        public boolean isFinished() {
                return translationController.atGoal() &&
                                strafeController.atGoal() &&
                                rotationController.atGoal();
        }
}