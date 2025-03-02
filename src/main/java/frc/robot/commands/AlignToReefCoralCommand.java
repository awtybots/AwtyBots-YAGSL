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
        System.out.println("AlignToReefCommand started - Aligning to " + (alignLeft ? "Left" : "Right")
                + " Reef. Target Distance: " + targetDistanceMeters + " meters");
    }

    @Override
    public void execute() {
        // ✅ Update PID Constants from SmartDashboard
        distancePID.setP(SmartDashboard.getNumber("PID/Distance kP", distancePID.getP()));
        distancePID.setI(SmartDashboard.getNumber("PID/Distance kI", distancePID.getI()));
        distancePID.setD(SmartDashboard.getNumber("PID/Distance kD", distancePID.getD()));

        strafePID.setP(SmartDashboard.getNumber("PID/Strafe kP", strafePID.getP()));
        strafePID.setI(SmartDashboard.getNumber("PID/Strafe kI", strafePID.getI()));
        strafePID.setD(SmartDashboard.getNumber("PID/Strafe kD", strafePID.getD()));

        rotationPID.setP(SmartDashboard.getNumber("PID/Rotation kP", rotationPID.getP()));
        rotationPID.setI(SmartDashboard.getNumber("PID/Rotation kI", rotationPID.getI()));
        rotationPID.setD(SmartDashboard.getNumber("PID/Rotation kD", rotationPID.getD()));

        Optional<double[]> errors = vision.getAlignmentErrors(alignLeft);

        if (errors.isPresent()) {
            hasValidTarget = true;
            double[] errorArray = errors.get();
            double targetYaw = errorArray[0];
            double distanceError = errorArray[1];

            // ✅ Dynamically select left/right offset
            double lateralOffset = alignLeft ? Constants.VisionConstants.Coral.leftOffsetMeters
                    : Constants.VisionConstants.Coral.rightOffsetMeters;

            // ✅ Compute PID outputs
            double forwardSpeed = distancePID.calculate(distanceError);
            double strafeSpeed = strafePID.calculate(lateralOffset);
            double rotationSpeed = rotationPID.calculate(targetYaw);

            // ✅ Add a deadband to prevent jittering
            if (Math.abs(forwardSpeed) < 0.02)
                forwardSpeed = 0;
            if (Math.abs(strafeSpeed) < 0.02)
                strafeSpeed = 0;
            if (Math.abs(rotationSpeed) < 0.02)
                rotationSpeed = 0;

            // ✅ Apply speeds using PID
            swerve.drive(new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed));

            // ✅ Send debug info to SmartDashboard
            SmartDashboard.putBoolean("Reef/Has Valid Target", true);
            SmartDashboard.putNumber("Reef/Target Yaw", targetYaw);
            SmartDashboard.putNumber("Reef/Distance Error", distanceError);
            SmartDashboard.putNumber("Reef/Lateral Offset", lateralOffset);
            SmartDashboard.putNumber("Reef/PID-Forward Speed", forwardSpeed);
            SmartDashboard.putNumber("Reef/PID-Strafe Speed", strafeSpeed);
            SmartDashboard.putNumber("Reef/PID-Rotation Speed", rotationSpeed);

        } else if (hasValidTarget) {
            // Maintain last valid movement (small corrections)
            swerve.drive(new ChassisSpeeds(
                    distancePID.calculate(0),
                    strafePID.calculate(0),
                    rotationPID.calculate(0)));
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
    }

    @Override
    public boolean isFinished() {
        return distancePID.atSetpoint() && strafePID.atSetpoint() && rotationPID.atSetpoint();
    }
}
