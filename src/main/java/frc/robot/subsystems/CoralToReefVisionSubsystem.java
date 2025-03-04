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

    public Optional<PhotonTrackedTarget> getBestTarget() {
        PhotonPipelineResult result = camera.getLatestResult(); // Always get the latest frame

        if (!result.hasTargets()) {
            if (Constants.DebugMode) {
                System.out.println("[Vision] No AprilTag targets found.");
            }
            return Optional.empty();
        }

        PhotonTrackedTarget target = result.getBestTarget();
        if (Constants.DebugMode) {
            System.out.println("[Vision] Found AprilTag - ID: " + target.getFiducialId() +
                    " | Yaw: " + target.getYaw() +
                    " | Pitch: " + target.getPitch() +
                    " | Area: " + target.getArea());
        }
        return Optional.of(target);
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
            int tagID = target.getFiducialId(); // Get detected AprilTag ID
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID); // Get tag pose from field layout

            if (tagPose.isEmpty()) {
                System.out.println("[Vision] AprilTag ID " + tagID + " has no known pose in the field layout!");
                return Optional.empty();
            }

            double aprilTagHeight = tagPose.get().getZ(); // Get the AprilTag's height (Z-axis)
            

            // Compute Distance to Target
            double targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.Coral.cameraMountZ, // Camera height in meters
                    aprilTagHeight, // Dynamic AprilTag height
                    Units.degreesToRadians(Constants.VisionConstants.Coral.cameraMountAngle), // Camera mount angle
                    Units.degreesToRadians(target.getPitch()));

            // Compute distance error
            double distanceError = targetRange - Constants.VisionConstants.Coral.targetDistanceMeters;

            // Left/Right Offset Adjustment
            double lateralOffset = alignLeft ? Constants.VisionConstants.Coral.leftOffsetMeters
                    : Constants.VisionConstants.Coral.rightOffsetMeters;

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
    }

    @Override
    public void simulationPeriodic() {
        logAprilTagData();
        updatePoseEstimation();
    }
}