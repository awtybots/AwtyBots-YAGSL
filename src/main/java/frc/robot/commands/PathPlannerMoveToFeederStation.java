package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;

import java.util.List;
import java.util.Optional;

public class PathPlannerMoveToFeederStation extends Command {
    private final SwerveSubsystem swerve;
    private final CoralToReefVisionSubsystem vision;
    private final Pose2d targetPose;
    private boolean pathStarted = false;

    public PathPlannerMoveToFeederStation(SwerveSubsystem swerve, CoralToReefVisionSubsystem vision,
            Pose2d targetPose) {
        this.swerve = swerve;
        this.vision = vision;
        this.targetPose = targetPose;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        Optional<Pose2d> visionPoseOpt = vision.getEstimatedPose();

        if (visionPoseOpt.isEmpty()) {
            System.out.println("[MoveToFeeder] No vision detected, backing up to get vision...");

            // Move back until we detect an AprilTag, then update odometry
            new BackupUntilVisionCommand(swerve, vision).schedule();
            return;
        } else {
            System.out.println("[MoveToFeeder] Vision Pose Found, Updating Odometry.");
            swerve.updateOdometry(visionPoseOpt.get());
        }

        // Now proceed with generating the path to the feeder station
        Pose2d startPose = swerve.getPose(); // Use the latest odometry pose

        // Create waypoints dynamically
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, targetPose);

        // Define path constraints from settings.json
        PathConstraints constraints = swerve.getPathConstraintsFromSettings();

        // Generate the dynamic path
        PathPlannerPath dynamicPath = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, targetPose.getRotation()));

        dynamicPath.preventFlipping = true; // Prevent unnecessary field mirroring

        // Start following the path
        AutoBuilder.followPath(dynamicPath).schedule();
        pathStarted = true;
    }

    @Override
    public boolean isFinished() {
        return pathStarted; // Ends once the path is started
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[MoveToFeeder] Path execution finished.");
    }
}
