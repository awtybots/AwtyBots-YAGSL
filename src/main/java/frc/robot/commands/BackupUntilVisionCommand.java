package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;

import java.util.Optional;

public class BackupUntilVisionCommand extends Command {
    private final SwerveSubsystem swerve;
    private final CoralToReefVisionSubsystem vision;
    private boolean visionPoseFound = false;

    private static final double BACKUP_SPEED = -0.6; // Negative for backward motion
    private static final double MAX_BACKUP_DISTANCE = 2; // Max meters to back up before giving up
    private Pose2d initialPose;

    public BackupUntilVisionCommand(SwerveSubsystem swerve, CoralToReefVisionSubsystem vision) {
        this.swerve = swerve;
        this.vision = vision;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        initialPose = swerve.getPose();
        visionPoseFound = false;
        System.out.println("[BackupUntilVision] Started backing up.");
    }

    @Override
    public void execute() {
        // Check if vision has found a valid AprilTag pose
        Optional<Pose2d> visionPoseOpt = vision.getEstimatedPose();
        if (visionPoseOpt.isPresent()) {
            swerve.updateOdometry(visionPoseOpt.get());
            visionPoseFound = true;
            System.out.println("[BackupUntilVision] Vision Pose Found! Updated Odometry: " + visionPoseOpt.get());
            return;
        }

        // Move backward until we reach the max distance
        double distanceTraveled = swerve.getPose().getTranslation().getDistance(initialPose.getTranslation());
        if (distanceTraveled >= MAX_BACKUP_DISTANCE) {
            System.out.println("[BackupUntilVision] Reached max backup distance, stopping.");
            visionPoseFound = true; // Exit the command even if no vision was found
        } else {
            swerve.drive(new ChassisSpeeds(BACKUP_SPEED, 0, 0));
        }
    }

    @Override
    public void end(boolean interrupted) {
        swerve.drive(new ChassisSpeeds(0, 0, 0)); // Stop moving
        System.out.println("[BackupUntilVision] Stopped.");
    }

    @Override
    public boolean isFinished() {
        return visionPoseFound; // Stop as soon as we get a vision update
    }
}
