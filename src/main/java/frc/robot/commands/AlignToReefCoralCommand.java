package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;
import java.util.Map;

public class AlignToReefCoralCommand extends Command {
        private final SwerveSubsystem swerve;
        private final CoralToReefVisionSubsystem vision;
        private final boolean alignLeft;

        // PID controllers for forward translation, strafing, and rotation
        private final ProfiledPIDController translationController;
        private final ProfiledPIDController strafeController;
        private final ProfiledPIDController rotationController;

        private boolean hasValidTarget = false;
        private Pose2d targetPose;

        public AlignToReefCoralCommand(SwerveSubsystem swerve, CoralToReefVisionSubsystem vision, boolean alignLeft) {
                this.swerve = swerve;
                this.vision = vision;
                this.alignLeft = alignLeft;

                addRequirements(swerve, vision);

                // Create PID controllers with your existing gains and motion constraints,
                // but update the tolerances to be tighter.
                translationController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.TRANSLATION_kP,
                                Constants.VisionConstants.Coral.TRANSLATION_kI,
                                Constants.VisionConstants.Coral.TRANSLATION_kD,
                                Constants.VisionConstants.Coral.TRANSLATION_CONSTRAINTS);
                // Set tolerance to ~1 inch (0.0254 m)
                translationController.setTolerance(0.0254);

                strafeController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.STRAFE_kP,
                                Constants.VisionConstants.Coral.STRAFE_kI,
                                Constants.VisionConstants.Coral.STRAFE_kD,
                                Constants.VisionConstants.Coral.STRAFE_CONSTRAINTS);
                strafeController.setTolerance(Constants.VisionConstants.Coral.STRAFE_TOLERANCE);

                rotationController = new ProfiledPIDController(
                                Constants.VisionConstants.Coral.ROTATION_kP,
                                Constants.VisionConstants.Coral.ROTATION_kI,
                                Constants.VisionConstants.Coral.ROTATION_kD,
                                Constants.VisionConstants.Coral.ROTATION_CONSTRAINTS);
                // Set rotation tolerance to ~1 degree (in radians)
                rotationController.setTolerance(Math.toRadians(1));
                // Enable continuous input to handle angle wraparound
                rotationController.enableContinuousInput(-Math.PI, Math.PI);
        }

        @Override
        public void initialize() {
                // Update vision-based odometry first
                vision.updateOdometryWithVision();
                // Instead of using the detected tag directly, select the best scoring pose from
                // your predefined set.
                Pose2d currentPose = swerve.getPose();
                targetPose = getBestScoringPose(currentPose);
                if (targetPose == null) {
                        System.out.println("[AlignToReefCoralCommand] No valid scoring pose found. Stopping.");
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }
                hasValidTarget = true;

                System.out.println("[AlignToReefCoralCommand] STARTED");
                System.out.println(" - Aligning to: " + (alignLeft ? "LEFT" : "RIGHT") + " Reef");
                System.out.println(" - Target Pose: " + targetPose);
        }

        @Override
        public void execute() {
                Pose2d currentPose = swerve.getPose();
                // Recalculate target in case the robot’s position has changed
                targetPose = getBestScoringPose(currentPose);
                if (targetPose == null) {
                        System.out.println(
                                        "[AlignToReefCoralCommand] No valid scoring pose found during execute. Stopping.");
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }

                // Calculate the error in translation (distance and lateral offset) and
                // rotation.
                double targetDistance = currentPose.getTranslation().getDistance(targetPose.getTranslation());
                double lateralOffset = currentPose.getTranslation().getY() - targetPose.getTranslation().getY();

                double forwardSpeed = translationController.calculate(targetDistance, 0);
                double strafeSpeed = strafeController.calculate(lateralOffset, 0);
                double rotationPIDOutput = rotationController.calculate(swerve.getGyroYaw(),
                                targetPose.getRotation().getDegrees());

                // Zero out commands if the error is within tolerance.
                if (targetDistance < translationController.getPositionTolerance()) {
                        forwardSpeed = 0;
                        translationController.reset(0);
                }
                if (Math.abs(lateralOffset) < strafeController.getPositionTolerance()) {
                        strafeSpeed = 0;
                        strafeController.reset(0);
                }
                if (Math.abs(swerve.getGyroYaw() - targetPose.getRotation().getDegrees()) < Math
                                .toDegrees(rotationController.getPositionTolerance())) {
                        rotationPIDOutput = 0;
                        rotationController.reset(targetPose.getRotation().getDegrees());
                }

                // Clamp speeds to your maximum allowed values.
                forwardSpeed = MathUtil.clamp(forwardSpeed,
                                -Constants.VisionConstants.Coral.maxForwardSpeed,
                                Constants.VisionConstants.Coral.maxForwardSpeed);
                strafeSpeed = MathUtil.clamp(strafeSpeed,
                                -Constants.VisionConstants.Coral.maxStrafeSpeed,
                                Constants.VisionConstants.Coral.maxStrafeSpeed);
                rotationPIDOutput = MathUtil.clamp(rotationPIDOutput,
                                -Constants.VisionConstants.Coral.maxRotationSpeed,
                                Constants.VisionConstants.Coral.maxRotationSpeed);

                System.out.println("[AlignToReefCoralCommand] EXECUTE");
                System.out.println("Current Pose: " + currentPose);
                System.out.println("Target Pose: " + targetPose);
                System.out.println("Target Distance: " + targetDistance);
                System.out.println("Lateral Offset: " + lateralOffset);
                System.out.println("Forward Speed: " + forwardSpeed);
                System.out.println("Strafe Speed: " + strafeSpeed);
                System.out.println("Rotation Speed: " + rotationPIDOutput);

                SmartDashboard.putNumber("PID-Vision/Forward Speed", forwardSpeed);
                SmartDashboard.putNumber("PID-Vision/Strafe Speed", strafeSpeed);
                SmartDashboard.putNumber("PID-Vision/Rotation Speed", rotationPIDOutput);

                swerve.drive(forwardSpeed, strafeSpeed, rotationPIDOutput);
        }

        @Override
        public void end(boolean interrupted) {
                System.out.println("[AlignToReefCoralCommand] END called. Interrupted: " + interrupted);
                hasValidTarget = false;
                swerve.stop();
        }

        @Override
        public boolean isFinished() {
                boolean finished = translationController.atGoal() &&
                                strafeController.atGoal() &&
                                rotationController.atGoal();
                System.out.println("[AlignToReefCoralCommand] isFinished: " + finished);
                return finished;
        }

        /**
         * Helper method that selects the best scoring pose from your predefined set.
         * It uses the robot’s current odometry to determine which of the left or right
         * scoring positions (from the appropriate alliance) is closest.
         */
        private Pose2d getBestScoringPose(Pose2d currentPose) {
                Map<Integer, Pose2d[]> scoringPoses;
                Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
                if (alliance == Alliance.Red) {
                        scoringPoses = Constants.VisionConstants.Coral.redReefScoringPoses;
                } else if (alliance == Alliance.Blue) {
                        scoringPoses = Constants.VisionConstants.Coral.blueReefScoringPoses;
                } else {
                        return null;
                }
                Pose2d bestPose = null;
                double bestDistance = Double.MAX_VALUE;
                for (Pose2d[] poses : scoringPoses.values()) {
                        // Choose the left or right bar pose based on alignLeft
                        Pose2d candidate = alignLeft ? poses[0] : poses[1];
                        double distance = candidate.getTranslation().getDistance(currentPose.getTranslation());
                        if (distance < bestDistance) {
                                bestDistance = distance;
                                bestPose = candidate;
                        }
                }
                return bestPose;
        }
}
