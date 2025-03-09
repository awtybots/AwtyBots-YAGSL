// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class MoveForwardCommand extends Command {
    private final SwerveSubsystem swerve;
    private final double forwardDistanceTarget; // Target distance in meters
    private Pose2d initialPose;
    private final PIDController distancePID;

    private static final double MAX_SPEED = 1.5; // Max speed in meters/sec
    private static final double MIN_SPEED = 0.2; // Minimum speed to prevent stopping early

    public MoveForwardCommand(SwerveSubsystem swerve, double forwardDistanceTarget) {
        this.swerve = swerve;
        this.forwardDistanceTarget = forwardDistanceTarget;
        addRequirements(swerve);

        // Create a PID controller for distance control
        distancePID = new PIDController(2.0, 0.0, 0.4); // Tune kP, kI, kD
        distancePID.setTolerance(0.05); // Stop within 5 cm of target
    }

    @Override
    public void initialize() {
        initialPose = swerve.getPose(); // Store the starting position
        distancePID.setSetpoint(forwardDistanceTarget); // Target distance
        System.out.println("[MoveForwardCommand] Moving forward: " + forwardDistanceTarget + " meters.");
    }

    @Override
    public void execute() {
        Pose2d currentPose = swerve.getPose();

        // Calculate distance traveled in meters
        double distanceTraveled = currentPose.getTranslation().getDistance(initialPose.getTranslation());
        double error = forwardDistanceTarget - distanceTraveled; // How much further we need to go

        // PID calculates speed based on distance left
        double speed = distancePID.calculate(distanceTraveled);

        // Gradually reduce speed as we approach the target
        double speedFactor = Math.max(error / forwardDistanceTarget, 0.2); // Min 20% speed near target
        speed *= speedFactor;

        // Clamp speed to prevent excessive acceleration
        speed = Math.max(-MAX_SPEED, Math.min(MAX_SPEED, speed));

        // Ensure it never goes below the minimum required speed unless it's stopping
        if (!distancePID.atSetpoint()) {
            speed = Math.copySign(Math.max(Math.abs(speed), MIN_SPEED), speed);
        } else {
            speed = 0; // Stop if we're at the setpoint
        }

        swerve.drive(new ChassisSpeeds(speed, 0, 0)); // Move forward with controlled speed
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0)); // Stop movement
        System.out.println("[MoveForwardCommand] Reached target distance.");
    }

    @Override
    public boolean isFinished() {
        return distancePID.atSetpoint(); // Stop when within tolerance
    }
}