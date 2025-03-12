package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;
import frc.robot.Constants;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.Optional;

public class AlignToReefCoralCommand extends Command {
        private final SwerveSubsystem swerve;
        private final CoralToReefVisionSubsystem vision;
        private final boolean alignLeft;
        private final double desiredRotationDegrees; // Target rotation angle

        private final ProfiledPIDController translationController;
        private final ProfiledPIDController strafeController;
        private final ProfiledPIDController rotationController;

        private boolean hasValidTarget = false;

        public AlignToReefCoralCommand(SwerveSubsystem swerve, CoralToReefVisionSubsystem vision, boolean alignLeft,
                        double desiredRotationDegrees) {
                this.swerve = swerve;
                this.vision = vision;
                this.alignLeft = alignLeft;
                this.desiredRotationDegrees = desiredRotationDegrees;

                addRequirements(swerve, vision);

                // Profiled PID Controllers for smooth control
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

                // Set the PID controller setpoints
                translationController.setGoal(Constants.VisionConstants.Coral.DISTANCE_THRESHOLD);
                strafeController.setGoal(alignLeft ? Constants.VisionConstants.Coral.LEFT_OFFSET
                                : Constants.VisionConstants.Coral.RIGHT_OFFSET);
                rotationController.setGoal(desiredRotationDegrees);
        }

        @Override
        public void initialize() {
                System.out.println("[AlignToReefCoralCommand] STARTED");
                System.out.println(" - Aligning to: " + (alignLeft ? "LEFT" : "RIGHT") + " Reef");
                System.out.println(" - Target Rotation Angle: " + desiredRotationDegrees + " degrees");

                Optional<double[]> alignmentErrorsOpt = vision.getAlignmentErrors();
                if (alignmentErrorsOpt.isEmpty()) {
                        System.out.println(" - No valid target found, canceling command.");
                        cancel();
                        return;
                }

                hasValidTarget = true;
        }

        @Override
        public void execute() {
                Optional<double[]> alignmentErrorsOpt = vision.getAlignmentErrors();
                if (alignmentErrorsOpt.isEmpty()) {
                        hasValidTarget = false;
                        swerve.stop();
                        SmartDashboard.putBoolean("Vision/03 Valid-Target", hasValidTarget);
                        return;
                }

                double[] errors = alignmentErrorsOpt.get();
                double currentYaw = errors[0]; // Current rotation error in degrees
                double targetDistance = errors[1]; // Distance in meters
                double lateralOffset = errors[2]; // Side-to-side error

                // Compute First Derivative (Rate of Change of Yaw)
                double yawRate = (-8.3728 * lateralOffset) - 69.9854;

                // Adjust targetYaw dynamically using calculus-based correction
                double correctedYaw = currentYaw + (yawRate * 0.01); // Small time-step factor

                // Determine strafe setpoint based on alignment side
                double strafeSetpoint = alignLeft
                                ? Constants.VisionConstants.Coral.LEFT_OFFSET
                                : Constants.VisionConstants.Coral.RIGHT_OFFSET;

                // PID Calculations
                double rotationPIDOutput = rotationController.calculate(correctedYaw, desiredRotationDegrees);
                double forwardSpeed = translationController.calculate(targetDistance,
                                Constants.VisionConstants.Coral.DISTANCE_THRESHOLD);
                double strafeSpeed = strafeController.calculate(lateralOffset, strafeSetpoint);

                // Apply tolerance-based stopping
                if (Math.abs(targetDistance
                                - Constants.VisionConstants.Coral.DISTANCE_THRESHOLD) < Constants.VisionConstants.Coral.TRANSLATION_TOLERANCE) {
                        forwardSpeed = 0; // Stop when within translation tolerance
                }
                if (Math.abs(lateralOffset - strafeSetpoint) < Constants.VisionConstants.Coral.STRAFE_TOLERANCE) {
                        strafeSpeed = 0; // Stop when within strafe tolerance
                }
                if (Math.abs(correctedYaw
                                - desiredRotationDegrees) < Constants.VisionConstants.Coral.ROTATION_THRESHOLD) {
                        rotationPIDOutput = 0; // Stop rotating when within rotation threshold
                }

                // Enforce speed limits
                forwardSpeed = MathUtil.clamp(forwardSpeed, -Constants.VisionConstants.Coral.maxForwardSpeed,
                                Constants.VisionConstants.Coral.maxForwardSpeed);
                strafeSpeed = MathUtil.clamp(strafeSpeed, -Constants.VisionConstants.Coral.maxStrafeSpeed,
                                Constants.VisionConstants.Coral.maxStrafeSpeed);
                rotationPIDOutput = MathUtil.clamp(rotationPIDOutput, -Constants.VisionConstants.Coral.maxRotationSpeed,
                                Constants.VisionConstants.Coral.maxRotationSpeed);

                // Log values for debugging
                SmartDashboard.putNumber("Vision/18 Updated Yaw Correction (w/ Derivative)", correctedYaw);
                SmartDashboard.putNumber("Vision/19 Yaw Rate (dYaw/dL)", yawRate);
                SmartDashboard.putNumber("Vision/20 Desired Rotation Setpoint", desiredRotationDegrees);
                SmartDashboard.putBoolean("Vision/03 Valid-Target", hasValidTarget);
                SmartDashboard.putNumber("PID-Vision/10 PID-Forward Speed", forwardSpeed);
                SmartDashboard.putNumber("PID-Vision/11 PID-Strafe Speed", strafeSpeed);
                SmartDashboard.putNumber("PID-Vision/12 PID-Rotation Speed", rotationPIDOutput);
                SmartDashboard.putNumber("Vision/04 Yaw (degrees)", correctedYaw);
                SmartDashboard.putNumber("Vision/05 Distance (m)", targetDistance);
                SmartDashboard.putNumber("Vision/06 Lateral Offset (m)", lateralOffset);

                // Apply movement
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
