// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

/** Aligns the robot to an AprilTag while the button is held */
public class AlignToAprilTagCommand extends Command {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;
    private final boolean alignLeft; // New parameter to decide which reef bar to align with

    private double lastKnownYaw = 0.0;
    private double lastKnownDistanceError = 0.0;
    private double lastKnownLateralOffset = 0.0;
    private boolean hasValidTarget = false;

    /**
     * Creates a new AlignToAprilTagCommand.
     *
     * @param swerve    The drivetrain subsystem.
     * @param vision    The vision subsystem.
     * @param alignLeft `true` to align with the left reef bar, `false` for the
     *                  right.
     */
    public AlignToAprilTagCommand(SwerveSubsystem swerve, VisionSubsystem vision, boolean alignLeft) {
        this.swerve = swerve;
        this.vision = vision;
        this.alignLeft = alignLeft; // Store alignment preference
        addRequirements(swerve, vision);
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {
        System.out.println("AlignToAprilTagCommand started - Aligning to " + (alignLeft ? "Left" : "Right") + " Reef");
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        Optional<double[]> errors = vision.getAlignmentErrors();
        Pose2d currentPose = swerve.getPose(); // Get robot's estimated pose

        if (errors.isPresent()) {
            // ✅ We have a valid target!
            double[] errorArray = errors.get();
            double targetYaw = errorArray[0]; // Yaw error from AprilTag
            double distanceError = errorArray[1]; // Distance to target in meters

            // Select the appropriate lateral offset based on the chosen reef bar
            double lateralOffset = alignLeft ? VisionConstants.leftOffsetMeters : VisionConstants.rightOffsetMeters;

            // Store last known valid values (so we can use them later if we lose the tag)
            lastKnownYaw = targetYaw;
            lastKnownDistanceError = distanceError;
            lastKnownLateralOffset = lateralOffset;
            hasValidTarget = true; // ✅ We now have a valid target!

            // Use **latest** camera values for alignment
            useVisionForAlignment(targetYaw, distanceError, lateralOffset, currentPose);

        } else if (hasValidTarget) {
            // No new AprilTag, but we have last known values → Use last valid movement
            // values
            swerve.drive(
                    new ChassisSpeeds(lastKnownDistanceError * 0.1, lastKnownLateralOffset * 0.1, lastKnownYaw * 0.01));

        } else {
            // No valid target, no previous values → Stop the robot
            swerve.drive(new ChassisSpeeds(0, 0, 0));
            hasValidTarget = false;
        }
    }

    /** Called once the command ends or is interrupted. */
    @Override
    public void end(boolean interrupted) {
        // Reset stored AprilTag data
        hasValidTarget = false;
        lastKnownYaw = 0.0;
        lastKnownDistanceError = 0.0;
        lastKnownLateralOffset = 0.0;

        // Stop the robot
        swerve.drive(new ChassisSpeeds(0, 0, 0));

        System.out.println("AlignToAprilTagCommand ended - Resetting data");
    }

    /**
     * Returns true when the command should end (never ends while button is held).
     */
    @Override
    public boolean isFinished() {
        return false;
    }

    private void useVisionForAlignment(double targetYaw, double distanceError, double lateralOffset,
            Pose2d currentPose) {
        // Get the current heading from the gyro
        double currentGyroYaw = swerve.getGyroYaw();

        // Compute the corrected yaw error using both the camera and gyro
        double yawCorrection = targetYaw - currentGyroYaw;

        // Pose-based correction (field-centric)
        double poseErrorX = VisionConstants.targetDistanceMeters - currentPose.getX(); // ✅ Now used!
        double poseErrorY = lateralOffset;

        // Apply rotation correction using gyro + camera yaw
        double rotationSpeed = (Math.abs(yawCorrection) > VisionConstants.maxYawError)
                ? -yawCorrection * VisionConstants.VISION_TURN_kP
                : 0;

        // ✅ Now using poseErrorX for stopping at the target distance!
        double forwardSpeed = Math.abs(poseErrorX) > VisionConstants.distance_tolerance
                ? poseErrorX * VisionConstants.VISION_DRIVE_kP
                : 0; // Stops forward/backward movement when close enough

        // Move left/right based on offsets & pose error
        double strafeSpeed = lateralOffset * 0.5 + poseErrorY * 0.2; // Fine-tune strafe speed

        // Convert to ChassisSpeeds (correct format for swerve)
        ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed);

        // Apply calculated movement
        swerve.drive(speeds);
    }

    // private void useGyroForAlignment() {
    // double holdRotation = -swerve.getGyroYaw() * 0.01; // Small correction to
    // hold steady
    // swerve.drive(new ChassisSpeeds(0, 0, holdRotation));
    // }
}
