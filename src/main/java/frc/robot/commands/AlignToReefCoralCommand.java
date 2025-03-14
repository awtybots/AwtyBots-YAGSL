package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
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
                        System.out.println("[AlignToReefCoralCommand] No valid vision target. Stopping.");
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }

                hasValidTarget = true;
        }

        @Override
        public void execute() {
                System.out.println("[AlignToReefCoralCommand] EXECUTE CALLED");
                Optional<Pair<Integer, Pose2d>> fieldPoseOpt = vision.getEstimatedFieldPose();

                if (fieldPoseOpt.isEmpty()) {
                        System.out.println("[AlignToReefCoralCommand] No vision target found. Stopping.");
                        hasValidTarget = false;
                        swerve.stop();
                        return;
                }

                int detectedAprilTagID = fieldPoseOpt.get().getFirst();
                Pose2d currentPose = swerve.getPose(); // Use Odometry Instead of Vision Pose

                // Use predefined AprilTag pose
                Pose2d targetPose = Constants.VisionConstants.Coral.getBestReefPose(detectedAprilTagID, alignLeft);

                // Compute Field-Centric Distance
                Translation2d translationError = targetPose.getTranslation().minus(currentPose.getTranslation());
                double targetDistance = translationError.getNorm();

                // Compute Robot Velocity Projection for Smoothed Movement
                ChassisSpeeds robotVelocity = swerve.getRobotVelocity();
                Translation2d velocityVector = new Translation2d(
                                robotVelocity.vxMetersPerSecond,
                                robotVelocity.vyMetersPerSecond);

                // Prevent division by zero
                double translationErrorNorm = translationError.getNorm();
                double velocityProjection = (translationErrorNorm > 0.001)
                                ? (velocityVector.getX() * translationError.getX()
                                                + velocityVector.getY() * translationError.getY())
                                                / translationErrorNorm
                                : 0.0;

                // ✅ Compute Forward Speed
                double forwardSpeed = translationController.calculate(targetDistance, 0) - velocityProjection;

                // ✅ Compute Strafe Speed
                double lateralOffset = currentPose.getTranslation().getY() - targetPose.getTranslation().getY();
                double strafeSpeed = strafeController.calculate(lateralOffset, 0);

                // ✅ Compute Rotation Speed
                double rotationPIDOutput = rotationController.calculate(
                                swerve.getGyroYaw(), targetPose.getRotation().getDegrees());

                // **Stop Conditions**
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

                // **Clamp Speeds**
                forwardSpeed = MathUtil.clamp(forwardSpeed,
                                -Constants.VisionConstants.Coral.maxForwardSpeed,
                                Constants.VisionConstants.Coral.maxForwardSpeed);
                strafeSpeed = MathUtil.clamp(strafeSpeed,
                                -Constants.VisionConstants.Coral.maxStrafeSpeed,
                                Constants.VisionConstants.Coral.maxStrafeSpeed);
                rotationPIDOutput = MathUtil.clamp(rotationPIDOutput,
                                -Constants.VisionConstants.Coral.maxRotationSpeed,
                                Constants.VisionConstants.Coral.maxRotationSpeed);

                // 📝 **LOGGING for Debugging**
                System.out.println("====== AlignToReefCoralCommand Debug ======");
                System.out.println("Current Pose: " + currentPose);
                System.out.println("Target Pose: " + targetPose);
                System.out.println("Target Distance: " + targetDistance);
                System.out.println("Lateral Offset: " + lateralOffset);
                System.out.println("Rotation Error: " + (swerve.getGyroYaw() - targetPose.getRotation().getDegrees()));
                System.out.println("Velocity Projection: " + velocityProjection);
                System.out.println("Forward Speed: " + forwardSpeed);
                System.out.println("Strafe Speed: " + strafeSpeed);
                System.out.println("Rotation Speed: " + rotationPIDOutput);
                System.out.println("========================================");

                SmartDashboard.putNumber("PID-Vision/10 PID-Forward Speed", forwardSpeed);
                SmartDashboard.putNumber("PID-Vision/11 PID-Strafe Speed", strafeSpeed);
                SmartDashboard.putNumber("PID-Vision/12 PID-Rotation Speed", rotationPIDOutput);

                // ✅ Apply movement
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

                System.out.println("[AlignToReefCoralCommand] isFinished() called. Result: " + finished);
                return finished;
        }
}
