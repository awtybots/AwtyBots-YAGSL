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

    double lateralOffset = 0.0;

    public Optional<double[]> getAlignmentErrors(boolean alignLeft) {
        return getBestTarget().flatMap(target -> {
            int tagID = target.getFiducialId();
            Optional<Pose3d> tagPose = fieldLayout.getTagPose(tagID);

            if (tagPose.isEmpty() || target.getArea() < 0.2 || Math.abs(target.getYaw()) > 30) {
                System.out.println("[Vision] Rejecting AprilTag ID " + tagID + " due to bad pose estimate.");
                return Optional.empty();
            }

            // ✅ Correct Distance Calculation (From PhotonVision Docs)
            double targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                    Constants.VisionConstants.Coral.cameraMountZ, // Camera height in meters
                    tagPose.get().getZ(), // AprilTag height
                    Units.degreesToRadians(Constants.VisionConstants.Coral.cameraMountAngle), // Camera mount angle
                    Units.degreesToRadians(target.getPitch())); // Use pitch from target

            // ✅ Correct Lateral Offset Calculation
            double lateralOffset = tagPose.get().getY() - Constants.VisionConstants.Coral.cameraMountY;

            // Adjust lateral offset based on left/right alignment
            lateralOffset += (alignLeft ? Constants.VisionConstants.Coral.leftOffsetMeters
                    : Constants.VisionConstants.Coral.rightOffsetMeters);

            return Optional.of(new double[] { target.getYaw(), targetRange, -lateralOffset, tagID });
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
        // Update the vision status on the dashboard
        SmartDashboard.putNumber("Vision/lateralOffset", lateralOffset);
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