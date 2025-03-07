// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;
import frc.robot.Constants;
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

    // ✅ Define allowed AprilTag IDs
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
    public AlignToReefCoralCommand(SwerveSubsystem swerve,
            CoralToReefVisionSubsystem vision, boolean alignLeft,
            double targetDistanceMeters) {
        this.swerve = swerve;
        this.vision = vision;
        this.alignLeft = alignLeft;
        this.targetDistanceMeters = targetDistanceMeters;
        addRequirements(swerve, vision);

        // ✅ Initialize PID Constants on SmartDashboard
        SmartDashboard.putNumber("PID-Vision/01 Distance kP", Constants.VisionConstants.Coral.DistancekP);
        SmartDashboard.putNumber("PID-Vision/02 Distance kI", Constants.VisionConstants.Coral.DistancekI);
        SmartDashboard.putNumber("PID-Vision/03 Distance kD", Constants.VisionConstants.Coral.DistancekD);

        SmartDashboard.putNumber("PID-Vision/04 Strafe kP", Constants.VisionConstants.Coral.StrafekP);
        SmartDashboard.putNumber("PID-Vision/05 Strafe kI", Constants.VisionConstants.Coral.StrafekI);
        SmartDashboard.putNumber("PID-Vision/06 Strafe kD", Constants.VisionConstants.Coral.StrafekD);

        SmartDashboard.putNumber("PID-Vision/07 Rotation kP", Constants.VisionConstants.Coral.RotationkP);
        SmartDashboard.putNumber("PID-Vision/08 Rotation kI", Constants.VisionConstants.Coral.RotationkI);
        SmartDashboard.putNumber("PID-Vision/09 Rotation kD", Constants.VisionConstants.Coral.RotationkD);

        // ✅ Initialize PID Controllers (will be updated dynamically in execute())
        distancePID = new PIDController(Constants.VisionConstants.Coral.DistancekP,
                Constants.VisionConstants.Coral.DistancekI,
                Constants.VisionConstants.Coral.DistancekD);
        strafePID = new PIDController(Constants.VisionConstants.Coral.StrafekP,
                Constants.VisionConstants.Coral.StrafekI,
                Constants.VisionConstants.Coral.StrafekD);
        rotationPID = new PIDController(Constants.VisionConstants.Coral.RotationkP,
                Constants.VisionConstants.Coral.RotationkI,
                Constants.VisionConstants.Coral.RotationkD);

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
        int detectedTagId = vision.getBestTargetTagID();

        // ✅ Ensure detected tag is in the valid set
        if (!VALID_APRILTAG_IDS.contains(detectedTagId)) {
            swerve.drive(new ChassisSpeeds(0, 0, 0)); // Stop movement if tag is not valid
            hasValidTarget = false;
            SmartDashboard.putBoolean("Vision/Valid-Target", hasValidTarget);
            return;
        }

        Optional<double[]> errors = vision.getAlignmentErrors(alignLeft);
        if (errors.isPresent()) {
            hasValidTarget = true;
            double[] errorArray = errors.get();
            double targetYaw = errorArray[0];
            double targetRange = Math.abs(errorArray[1]);
            double lateralOffset = errorArray[2];

            // ✅ Use PID controllers for smoother movement
            double rotationSpeed = rotationPID.calculate(targetYaw, 0);
            double forwardSpeed = distancePID.calculate(targetDistanceMeters - targetRange, 0);
            double strafeSpeed = (Math.abs(lateralOffset) > Constants.VisionConstants.Coral.strafeThreshold)
                    ? strafePID.calculate(lateralOffset, 0)
                    : 0;

            // ✅ Enforce max speed limits
            forwardSpeed = Math.max(-Constants.VisionConstants.Coral.maxForwardSpeed,
                    Math.min(Constants.VisionConstants.Coral.maxForwardSpeed, forwardSpeed));
            strafeSpeed = Math.max(-Constants.VisionConstants.Coral.maxStrafeSpeed,
                    Math.min(Constants.VisionConstants.Coral.maxStrafeSpeed, strafeSpeed));
            rotationSpeed = Math.max(-Constants.VisionConstants.Coral.maxRotationSpeed,
                    Math.min(Constants.VisionConstants.Coral.maxRotationSpeed, rotationSpeed));

            // ✅ Apply corrected movement values
            swerve.drive(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed));

            // ✅ Log PID values to SmartDashboard
            SmartDashboard.putBoolean("Vision/Valid-Target", hasValidTarget);
            SmartDashboard.putNumber("PID-Vision/10 PID-Forward Speed", forwardSpeed);
            SmartDashboard.putNumber("PID-Vision/11 PID-Strafe Speed", strafeSpeed);
            SmartDashboard.putNumber("PID-Vision/12 PID-Rotation Speed", rotationSpeed);
        } else {
            // No valid target → Stop the robot
            swerve.drive(new ChassisSpeeds(0, 0, 0));

            hasValidTarget = false;
            SmartDashboard.putBoolean("Vision/Valid-Target", hasValidTarget);
        }
    }

    @Override
    public void end(boolean interrupted) {
        hasValidTarget = false;
        swerve.drive(new ChassisSpeeds(0, 0, 0));
        System.out.println("AlignToReefCommand ended.");
    }

    @Override
    public boolean isFinished() {
        return distancePID.atSetpoint() && strafePID.atSetpoint() &&
                rotationPID.atSetpoint();
    }
}
