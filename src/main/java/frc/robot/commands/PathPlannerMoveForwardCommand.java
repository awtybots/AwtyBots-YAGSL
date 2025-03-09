package frc.robot.commands;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.CoralToReefVisionSubsystem;

import java.util.List;
import java.util.Optional;

public class PathPlannerMoveForwardCommand extends Command {
    private final SwerveSubsystem swerve;
    private final CoralToReefVisionSubsystem vision;
    private final double forwardDistanceTarget;
    private boolean pathStarted = false;

    public PathPlannerMoveForwardCommand(SwerveSubsystem swerve, CoralToReefVisionSubsystem vision,
            double forwardDistanceTarget) {
        this.swerve = swerve;
        this.vision = vision;
        this.forwardDistanceTarget = forwardDistanceTarget;
        addRequirements(swerve);
    }

    @Override
    public void initialize() {
        // Try to get the most accurate pose using vision first
        Optional<Pose2d> visionPoseOpt = vision.getEstimatedPose();

        if (visionPoseOpt.isPresent()) {
            swerve.updateOdometry(visionPoseOpt.get());
            System.out.println("[MoveForwardCommand] Updated Odometry from Vision: " + visionPoseOpt.get());
        } else {
            System.out.println("[MoveForwardCommand] No Vision Pose Found, Using Odometry.");
        }

        // Use the latest odometry pose
        Pose2d startPose = swerve.getPose();

        // Create target position forward from the current pose
        Pose2d targetPose = new Pose2d(
                startPose.getTranslation().plus(new Translation2d(forwardDistanceTarget, 0)), // Move forward
                startPose.getRotation() // Maintain current heading
        );

        // Create waypoints dynamically from the start to the target position
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(startPose, targetPose);

        // Load constraints dynamically from settings.json
        PathConstraints constraints = swerve.getPathConstraintsFromSettings();

        // Generate the path dynamically
        PathPlannerPath dynamicPath = new PathPlannerPath(
                waypoints,
                constraints,
                null,
                new GoalEndState(0.0, startPose.getRotation()) // Maintain orientation
        );

        dynamicPath.preventFlipping = true; // Prevent field mirroring

        // Start executing the dynamic path
        AutoBuilder.followPath(dynamicPath).schedule();
        pathStarted = true;

        System.out.println("[MoveForwardCommand] Started moving forward " + forwardDistanceTarget + " meters.");
    }

    @Override
    public boolean isFinished() {
        return pathStarted; // Ends immediately after scheduling the path
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("[MoveForwardCommand] Path execution finished.");
    }
}
