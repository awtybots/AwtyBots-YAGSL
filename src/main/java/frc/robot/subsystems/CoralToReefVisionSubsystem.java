package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
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
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Rotation3d;

public class CoralToReefVisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.Coral.limelightAprilTagCamera);
    private final AprilTagFieldLayout fieldLayout;
    private final Transform3d robotToCamera;
    private final PhotonPoseEstimator photonPoseEstimator;

    public CoralToReefVisionSubsystem() {
        // Load the 2025 AprilTag field layout
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        // Camera position relative to the robot
        robotToCamera = new Transform3d(
                Constants.VisionConstants.Coral.cameraMountX,
                Constants.VisionConstants.Coral.cameraMountY,
                Constants.VisionConstants.Coral.cameraMountZ,
                new Rotation3d(0, 0, 0));

        // Initialize pose estimator
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);
    }

    private PhotonTrackedTarget lastKnownTarget = null;

    public Optional<PhotonTrackedTarget> getBestTarget() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            lastKnownTarget = result.getBestTarget();
            return Optional.of(lastKnownTarget);
        }

        // Instead of returning the last known target, return an empty optional if no
        // valid target
        return Optional.empty();
    }

    /**
     * Returns the AprilTag ID of the best detected target, or -1 if no valid target
     * exists
     */
    public int getBestTargetTagID() {
        var targetOpt = getBestTarget();
        return targetOpt.map(PhotonTrackedTarget::getFiducialId).orElse(-1);
    }

    private double smoothedDistance = 0.0;
    private final double SMOOTHING_FACTOR = 0.8;

    public void logVisionData() {
        var targetOpt = getBestTarget();

        if (targetOpt.isPresent()) {
            PhotonTrackedTarget target = targetOpt.get();
            Transform3d cameraToTarget = target.getBestCameraToTarget(); // Use PhotonVision's 3D transform

            double rawDistance = cameraToTarget.getTranslation().getNorm(); // Accurate distance
            smoothedDistance = (SMOOTHING_FACTOR * rawDistance) + ((1 - SMOOTHING_FACTOR) * smoothedDistance);

            double normalLateralOffset = cameraToTarget.getY(); // Raw side-to-side alignment error

            // ✅ Log AprilTag Tracking Data
            SmartDashboard.putBoolean("Vision/01 AprilTag Found", true);
            SmartDashboard.putNumber("Vision/02 AprilTag ID", target.getFiducialId());
            SmartDashboard.putNumber("Vision/04 Yaw (degrees)", target.getYaw());
            SmartDashboard.putNumber("Vision/05 Distance (m)", smoothedDistance); // Use smoothed distance!
            SmartDashboard.putNumber("Vision/06 Raw Lateral Offset (m)", normalLateralOffset);
            SmartDashboard.putNumber("Vision/07 Lateral Offset Target (Left Align)",
                    Constants.VisionConstants.Coral.leftOffsetMeters);
            SmartDashboard.putNumber("Vision/08 Lateral Offset Target (Right Align)",
                    Constants.VisionConstants.Coral.rightOffsetMeters);
        } else {
            SmartDashboard.putBoolean("Vision/01 AprilTag Found", false);
        }

        // ✅ Log Estimated Global Pose (3D)
        var estimatedGlobalPoseOpt = getEstimatedGlobalPose();
        if (estimatedGlobalPoseOpt.isPresent()) {
            EstimatedRobotPose estimatedPose = estimatedGlobalPoseOpt.get();
            Pose3d robotPose3d = estimatedPose.estimatedPose;

            SmartDashboard.putBoolean("Vision/09 Estimated Global Pose Found", true);
            SmartDashboard.putNumber("Vision/10 Estimated X (m)", robotPose3d.getX());
            SmartDashboard.putNumber("Vision/11 Estimated Y (m)", robotPose3d.getY());
            SmartDashboard.putNumber("Vision/12 Estimated Rotation (deg)", robotPose3d.getRotation().getZ());
            SmartDashboard.putNumber("Vision/13 Estimated Timestamp (s)", estimatedPose.timestampSeconds);
        } else {
            SmartDashboard.putBoolean("Vision/09 Estimated Global Pose Found", false);
        }

        // ✅ Log Estimated 2D Pose
        var estimatedPoseOpt = getEstimatedPose();
        if (estimatedPoseOpt.isPresent()) {
            Pose2d robotPose2d = estimatedPoseOpt.get();
            SmartDashboard.putBoolean("Vision/14 Estimated Pose2D Found", true);
            SmartDashboard.putNumber("Vision/15 Estimated Pose2D X (m)", robotPose2d.getX());
            SmartDashboard.putNumber("Vision/16 Estimated Pose2D Y (m)", robotPose2d.getY());
            SmartDashboard.putNumber("Vision/17 Estimated Pose2D Rotation (deg)",
                    robotPose2d.getRotation().getDegrees());
        } else {
            SmartDashboard.putBoolean("Vision/14 Estimated Pose2D Found", false);
        }
    }

    /** Returns alignment errors [yaw, distance, lateral offset, tag ID] */
    public Optional<double[]> getAlignmentErrors() {
        var targetOpt = getBestTarget();

        if (targetOpt.isPresent()) {
            PhotonTrackedTarget target = targetOpt.get();
            Transform3d cameraToTarget = target.getBestCameraToTarget(); // Use PhotonVision's 3D transform

            double targetYaw = target.getYaw(); // Rotation error in degrees
            double targetRange = cameraToTarget.getTranslation().getNorm(); // Distance in meters
            double lateralOffset = cameraToTarget.getY(); // Side-to-side alignment error

            return Optional.of(new double[] { targetYaw, targetRange, lateralOffset, target.getFiducialId() });
        }

        return Optional.empty(); // No valid target found
    }

    public Optional<EstimatedRobotPose> getEstimatedGlobalPose() {
        PhotonPipelineResult result = camera.getLatestResult();

        if (result.hasTargets()) {
            // Use PhotonVision's built-in estimator to calculate the pose
            Optional<EstimatedRobotPose> estimatedPose = photonPoseEstimator.update(result);
            return estimatedPose;
        }

        return Optional.empty(); // No valid vision target
    }

    public Optional<Pose2d> getEstimatedPose() {
        Optional<EstimatedRobotPose> estimatedPoseOpt = getEstimatedGlobalPose();

        if (estimatedPoseOpt.isPresent()) {
            Pose2d estimatedPose2d = estimatedPoseOpt.get().estimatedPose.toPose2d();
            return Optional.of(estimatedPose2d);
        }

        return Optional.empty();
    }

    private int loopCounter = 0;

    @Override
    public void periodic() {
        if (loopCounter % 3 == 0) { // Run every 3 cycles (~60ms instead of every 20ms)
            logVisionData();
        }
        loopCounter++;
    }

    @Override
    public void simulationPeriodic() {
        if (loopCounter % 3 == 0) { // Run every 3 cycles (~60ms instead of every 20ms)
            logVisionData();
        }
        loopCounter++;
    }
}
