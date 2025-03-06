// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.VisionConstants;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import java.util.Optional;
import java.util.Set;

/** Aligns the robot to an AprilTag while the button is held */
public class AlignToReefCoralCommand extends Command {
    private final SwerveSubsystem swerve;
    private final CoralToReefVisionSubsystem vision;
    private final boolean alignLeft;
    private final double targetDistanceMeters;

    // PID Controllers
    private final PIDController distancePID;
    private final PIDController strafePID;
    private final PIDController rotationPID;

    private boolean hasValidTarget = false;
    private static final Set<Integer> VALID_APRILTAG_IDS = Set.of(1, 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22);

    /**
     * Creates a new AlignToAprilTagCommand.
     *
     * @param swerve               The drivetrain subsystem.
     * @param vision               The vision subsystem.
     * @param alignLeft            `true` to align to left reef bar, `false` for the
     *                             right.
     * @param targetDistanceMeters Defines how close you want the robot to target
     * 
     */
    public AlignToReefCoralCommand(SwerveSubsystem swerve, CoralToReefVisionSubsystem vision, boolean alignLeft,
            double targetDistanceMeters) {
        this.swerve = swerve;
        this.vision = vision;
        this.alignLeft = alignLeft;
        this.targetDistanceMeters = targetDistanceMeters;
        addRequirements(swerve, vision);

        // ✅ Initialize PID Constants on SmartDashboard
        SmartDashboard.putNumber("PID/Distance kP", Constants.VisionConstants.Coral.DistancekP);
        SmartDashboard.putNumber("PID/Distance kI", Constants.VisionConstants.Coral.DistancekI);
        SmartDashboard.putNumber("PID/Distance kD", Constants.VisionConstants.Coral.DistancekD);

        SmartDashboard.putNumber("PID/Strafe kP", Constants.VisionConstants.Coral.StrafekP);
        SmartDashboard.putNumber("PID/Strafe kI", Constants.VisionConstants.Coral.StrafekI);
        SmartDashboard.putNumber("PID/Strafe kD", Constants.VisionConstants.Coral.StrafekD);

        SmartDashboard.putNumber("PID/Rotation kP", Constants.VisionConstants.Coral.RotationkP);
        SmartDashboard.putNumber("PID/Rotation kI", Constants.VisionConstants.Coral.RotationkI);
        SmartDashboard.putNumber("PID/Rotation kD", Constants.VisionConstants.Coral.RotationkD);

        // ✅ Initialize PID Controllers (will be updated dynamically in execute())
        distancePID = new PIDController(Constants.VisionConstants.Coral.DistancekP,
                Constants.VisionConstants.Coral.DistancekI, Constants.VisionConstants.Coral.DistancekD);
        strafePID = new PIDController(Constants.VisionConstants.Coral.StrafekP,
                Constants.VisionConstants.Coral.StrafekI, Constants.VisionConstants.Coral.StrafekD);
        rotationPID = new PIDController(Constants.VisionConstants.Coral.RotationkP,
                Constants.VisionConstants.Coral.RotationkI, Constants.VisionConstants.Coral.RotationkD);

        // ✅ Set Tolerances
        distancePID.setTolerance(Constants.VisionConstants.Coral.distance_tolerance);
        strafePID.setTolerance(Constants.VisionConstants.Coral.strafe_tolerance);
        rotationPID.setTolerance(Constants.VisionConstants.Coral.rotation_tolerance);

        distancePID.setSetpoint(targetDistanceMeters);
        strafePID.setSetpoint(0.0);
        rotationPID.setSetpoint(0.0);
    }

    @Override
    public void initialize() {
        System.out.println("[AlignToReefCoralCommand] STARTED");
        System.out.println(" - Aligning to: " + (alignLeft ? "LEFT" : "RIGHT") + " Reef");
        System.out.println(" - Target Distance: " + targetDistanceMeters + " meters");
    }

    @Override
    public void execute() {
        Optional<double[]> errors = vision.getAlignmentErrors(alignLeft);

        if (errors.isPresent()) {
            hasValidTarget = true;
            double[] errorArray = errors.get();
            double targetYaw = errorArray[0]; // Yaw error
            double targetRange = Math.abs(errorArray[1]); // ✅ Ensure distance is positive
            double lateralOffset = errorArray[2]; // ✅ Side-to-side offset
            int detectedTagId = (int) errorArray[3];

            // ✅ Ensure valid AprilTag ID
            if (!VALID_APRILTAG_IDS.contains(detectedTagId)) {
                swerve.drive(new ChassisSpeeds(0, 0, 0)); // Stop movement
                return;
            }

            // ✅ Fix Yaw: Stop turning when close to aligned
            double rotationSpeed = targetYaw * Constants.VisionConstants.Coral.RotationkP;
            if (Math.abs(targetYaw) < Constants.VisionConstants.Coral.rotation_tolerance) {
                rotationSpeed = 0; // 🔄 Stop rotating if within 2 degrees
            }

            // ✅ Fix Distance: Stop moving when close
            double distanceError = Constants.VisionConstants.Coral.targetDistanceMeters - targetRange;
            double forwardSpeed = distanceError * Constants.VisionConstants.Coral.DistancekP;
            if (Math.abs(distanceError) < Constants.VisionConstants.Coral.distance_tolerance) {
                forwardSpeed = 0; // 🔄 Stop moving if within 0.05m
            }

            // ✅ Fix Strafe: Keep the same logic
            double strafeSpeed = Math.abs(lateralOffset) > Constants.VisionConstants.Coral.strafeThreshold
                    ? -lateralOffset * Constants.VisionConstants.Coral.StrafekP
                    : 0;

            // ✅ Enforce max speed limits
            forwardSpeed = Math.max(-Constants.VisionConstants.Coral.maxForwardSpeed,
                    Math.min(Constants.VisionConstants.Coral.maxForwardSpeed, forwardSpeed));

            strafeSpeed = Math.max(-Constants.VisionConstants.Coral.maxStrafeSpeed,
                    Math.min(Constants.VisionConstants.Coral.maxStrafeSpeed, strafeSpeed));

            rotationSpeed = Math.max(-Constants.VisionConstants.Coral.maxRotationSpeed,
                    Math.min(Constants.VisionConstants.Coral.maxRotationSpeed, rotationSpeed));

            // ✅ Debug output
            System.out.println("[AlignToReefCoralCommand] Movement Outputs:");
            System.out.println(" - Target Yaw: " + targetYaw);
            System.out.println(" - Target Range: " + targetRange);
            System.out.println(" - Distance Error: " + distanceError);
            System.out.println(" - Lateral Offset: " + lateralOffset);
            System.out.println(" - PID Forward Speed: " + forwardSpeed);
            System.out.println(" - PID Strafe Speed: " + strafeSpeed);
            System.out.println(" - PID Rotation Speed: " + rotationSpeed);
            System.out.println(" - Expected Strafe Direction: " + (lateralOffset > 0 ? "Left" : "Right"));

            // ✅ Apply corrected movement values
            swerve.drive(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed));

            // ✅ Update Shuffleboard
            SmartDashboard.putBoolean("Reef/Has Valid Target", true);
            SmartDashboard.putNumber("Reef/Target Yaw", targetYaw);
            SmartDashboard.putNumber("Reef/Target Range", targetRange);
            SmartDashboard.putNumber("Reef/Distance Error", distanceError);
            SmartDashboard.putNumber("Reef/Lateral Offset", lateralOffset);
            SmartDashboard.putNumber("Reef/PID-Forward Speed", forwardSpeed);
            SmartDashboard.putNumber("Reef/PID-Strafe Speed", strafeSpeed);
            SmartDashboard.putNumber("Reef/PID-Rotation Speed", rotationSpeed);
        } else {
            // No valid target → Stop the robot
            swerve.drive(new ChassisSpeeds(0, 0, 0));
            hasValidTarget = false;
        }
    }

    @Override
    public void end(boolean interrupted) {
        hasValidTarget = false;
        swerve.drive(new ChassisSpeeds(0, 0, 0));
        System.out.println("AlignToReefCommand ended.");
        vision.resetLastKnownTarget();
    }

    @Override
    public boolean isFinished() {
        return distancePID.atSetpoint() && strafePID.atSetpoint() && rotationPID.atSetpoint();
    }
}
