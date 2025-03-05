package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import edu.wpi.first.math.util.Units;
import java.util.Optional;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class CoralToReefVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.Coral.limelightAprilTagCamera);
    private SwerveSubsystem swerve;

    private final AprilTagFieldLayout fieldLayout; // AprilTag field layout
    private final Transform3d robotToCamera; // Camera position relative to the robot
    private final PhotonPoseEstimator photonPoseEstimator; // Vision-based pose estimator

    public CoralToReefVisionSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;

        // Load the AprilTag field layout (2025 game)
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        // Define the camera's position relative to the robot (adjust values based on
        // actual setup)
        robotToCamera = new Transform3d(Constants.VisionConstants.Coral.cameraMountX,
                Constants.VisionConstants.Coral.cameraMountY, Constants.VisionConstants.Coral.cameraMountX,
                new Rotation3d(0, 0, 0));

        // Initialize the pose estimator using the camera and tag layout
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);
    }

    private PhotonTrackedTarget lastKnownTarget = null;

    public Optional<PhotonTrackedTarget> getBestTarget() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            lastKnownTarget = result.getBestTarget(); // Store the best target
            if (Constants.DebugMode) {
                System.out.println("[Vision] Found AprilTag - ID: " + lastKnownTarget.getFiducialId() +
                        " | Yaw: " + lastKnownTarget.getYaw() +
                        " | Pitch: " + lastKnownTarget.getPitch() +
                        " | Area: " + lastKnownTarget.getArea());
            }
            return Optional.of(lastKnownTarget);
        }

        // Return last known target if we lost sight of it
        if (lastKnownTarget != null) {
            return Optional.of(lastKnownTarget);
        }

        return Optional.empty();
    }

    public void resetLastKnownTarget() {
        lastKnownTarget = null;
    }

    public void logAprilTagData() {
        PhotonPipelineResult result = camera.getLatestResult(); // Always get latest

        // System.out.println("Logging latest AprilTag data...");

        if (result.hasTargets()) {
            PhotonTrackedTarget target = result.getBestTarget();

            SmartDashboard.putBoolean("AprilTag Found", true);
            SmartDashboard.putNumber("AprilTag ID", target.getFiducialId());
            SmartDashboard.putNumber("AprilTag Yaw", target.getYaw());
            SmartDashboard.putNumber("AprilTag Pitch", target.getPitch());
            SmartDashboard.putNumber("AprilTag Skew", target.getSkew());
            SmartDashboard.putNumber("AprilTag Area", target.getArea());

            // System.out.println("AprilTag Detected - ID: " + target.getFiducialId());
        } else {
            SmartDashboard.putBoolean("AprilTag Found", false);
        }

    }

    public Optional<double[]> getAlignmentErrors(boolean alignLeft) {
        return getBestTarget().flatMap(target -> {
            int tagID = target.getFiducialId();
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);

            if (tagPose.isEmpty() || target.getArea() < 0.2 || Math.abs(target.getYaw()) > 30) {
                System.out.println("[Vision] Rejecting AprilTag ID " + tagID + " due to bad pose estimate.");
                return Optional.empty();
            }

            double aprilTagHeight = tagPose.get().getZ(); // AprilTag's height (Z-axis)
            double aprilTagY = tagPose.get().getY(); // AprilTag's lateral offset (Y-axis)
            double aprilTagX = tagPose.get().getX(); // AprilTag's forward offset (X-axis)

            // Compute distance to target, considering camera position
            double targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.Coral.cameraMountZ, // Camera height in meters
                    aprilTagHeight, // Dynamic AprilTag height
                    Units.degreesToRadians(Constants.VisionConstants.Coral.cameraMountAngle), // Camera mount angle
                    Units.degreesToRadians(target.getPitch()));

            // **Offset Target Distance**
            // Since the camera is front-right, adjust the stop distance
            double adjustedTargetDistance = Constants.VisionConstants.Coral.targetDistanceMeters +
                    Constants.VisionConstants.Coral.cameraMountX;

            // Compute distance error with the adjusted stopping point
            double distanceError = targetRange - adjustedTargetDistance;

            // **Account for Lateral Offset**
            double yOffsetError = aprilTagY - Constants.VisionConstants.Coral.cameraMountY;

            // Adjust alignment based on left/right target selection
            double lateralOffset = (alignLeft ? Constants.VisionConstants.Coral.leftOffsetMeters
                    : Constants.VisionConstants.Coral.rightOffsetMeters) + yOffsetError;

            return Optional.of(new double[] { target.getYaw(), distanceError, lateralOffset, tagID });
        });
    }

    public void updatePoseEstimation() {
        var results = camera.getAllUnreadResults();
        if (results.isEmpty()) {
            return; // Prevents crash if no frames exist
        }

        // Use the latest result from the list
        PhotonPipelineResult result = results.get(results.size() - 1);

        if (!result.hasTargets()) {
            return; // No targets detected, do nothing
        }

        var poseEstimate = photonPoseEstimator.update(result);
        if (poseEstimate.isPresent()) {
            var poseEst = poseEstimate.get();
            swerve.addVisionMeasurement(poseEst.estimatedPose.toPose2d(), poseEst.timestampSeconds);
        }
    }

    @Override
    public void periodic() {
        updatePoseEstimation();
        logAprilTagData();

        // If we lost the target, continue aligning using the last known good pose
        if (lastKnownTarget != null && !camera.getLatestResult().hasTargets()) {
            SmartDashboard.putString("Vision Status", "Using Last Known Target");
        } else {
            SmartDashboard.putString("Vision Status", "Tracking New Target");
        }
    }

    @Override
    public void simulationPeriodic() {
        logAprilTagData();
        updatePoseEstimation();
    }
}