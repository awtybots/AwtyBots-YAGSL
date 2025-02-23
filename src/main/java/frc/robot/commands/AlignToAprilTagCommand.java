// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;

/** Aligns the robot to an AprilTag while the button is held */
public class AlignToAprilTagCommand extends Command {
    private final SwerveSubsystem swerve;
    private final VisionSubsystem vision;

    private double lastKnownYaw = 0.0;
    private double lastKnownDistanceError = 0.0;
    private double lastKnownLateralOffset = 0.0;
    private boolean hasValidTarget = false;

    /**
     * Creates a new AlignToAprilTagCommand.
     *
     * @param swerve The drivetrain subsystem.
     * @param vision The vision subsystem.
     */
    public AlignToAprilTagCommand(SwerveSubsystem swerve, VisionSubsystem vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve, vision);
    }

    /** Called when the command is initially scheduled. */
    @Override
    public void initialize() {
        System.out.println("AlignToAprilTagCommand started");
    }

    /** Called every time the scheduler runs while the command is scheduled. */
    @Override
    public void execute() {
        Optional<double[]> errors = vision.getAlignmentErrors();

        if (errors.isPresent()) {
            // ✅ We have a valid target!
            double[] errorArray = errors.get();
            double targetYaw = errorArray[0]; // Yaw error from AprilTag
            double distanceError = errorArray[1]; // Distance to target in meters
            double lateralOffset = errorArray[2]; // Left/right offset

            // Store last known valid values (so we can use them later if we lose the tag)
            lastKnownYaw = targetYaw;
            lastKnownDistanceError = distanceError;
            lastKnownLateralOffset = lateralOffset;
            hasValidTarget = true; // ✅ We now have a valid target!

            // Use **latest** camera values for alignment
            useVisionForAlignment(targetYaw, distanceError, lateralOffset);

        } else if (hasValidTarget) {
            // No new AprilTag, but we have last known values → Use gyro for stability
            useGyroForAlignment();

        } else {
            // No valid target, no previous values → Stop the robot
            swerve.drive(new ChassisSpeeds(0, 0, 0));
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

    private void useVisionForAlignment(double targetYaw, double distanceError, double lateralOffset) {
        // Get the current heading from the gyro
        double currentGyroYaw = swerve.getGyroYaw();

        // Compute the corrected yaw error using both the camera and gyro
        double yawCorrection = targetYaw - currentGyroYaw;

        // Apply rotation correction using gyro + camera yaw
        double rotationSpeed = (Math.abs(yawCorrection) > VisionConstants.maxYawError)
                ? -yawCorrection * VisionConstants.VISION_TURN_kP
                : 0;

        // Stop Forward Movement if within the Target Range
        double forwardSpeed = Math.abs(distanceError) > VisionConstants.distance_tolerance
                ? distanceError * VisionConstants.VISION_DRIVE_kP
                : 0; // Stops forward/backward movement when close enough

        // Move left/right based on offsets
        double strafeSpeed = lateralOffset * 0.5;

        // Convert to ChassisSpeeds (correct format for swerve)
        ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed);

        // Apply calculated movement
        swerve.drive(speeds);
    }

    private void useGyroForAlignment() {
        double holdRotation = -swerve.getGyroYaw() * 0.01; // Small correction to hold steady
        swerve.drive(new ChassisSpeeds(0, 0, holdRotation));
    }
}
