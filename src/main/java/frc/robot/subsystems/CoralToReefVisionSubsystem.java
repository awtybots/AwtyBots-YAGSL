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
            double adjustedLateralOffsetLeft = normalLateralOffset + Constants.VisionConstants.Coral.leftOffsetMeters;
            double adjustedLateralOffsetRight = normalLateralOffset - Constants.VisionConstants.Coral.rightOffsetMeters;

            // ✅ Log values to SmartDashboard
            SmartDashboard.putBoolean("Vision/01 AprilTag Found", true);
            SmartDashboard.putNumber("Vision/02 AprilTag ID", target.getFiducialId());
            SmartDashboard.putNumber("Vision/04 Yaw (degrees)", target.getYaw());
            SmartDashboard.putNumber("Vision/05 Distance (m)", smoothedDistance); // Use smoothed distance!
            SmartDashboard.putNumber("Vision/06 Raw Lateral Offset (m)", normalLateralOffset);
            SmartDashboard.putNumber("Vision/07 Adjusted Lateral Offset (Left Align)", adjustedLateralOffsetLeft);
            SmartDashboard.putNumber("Vision/08 Adjusted Lateral Offset (Right Align)", adjustedLateralOffsetRight);
        } else {
            SmartDashboard.putBoolean("Vision/01 AprilTag Found", false);
        }
    }

    /** Returns alignment errors [yaw, distance, lateral offset, tag ID] */
    public Optional<double[]> getAlignmentErrors(boolean alignLeft) {
        var targetOpt = getBestTarget();

        if (targetOpt.isPresent()) {
            PhotonTrackedTarget target = targetOpt.get();
            Transform3d cameraToTarget = target.getBestCameraToTarget(); // Use PhotonVision's 3D transform

            double targetYaw = target.getYaw(); // Rotation error in degrees
            double targetRange = cameraToTarget.getTranslation().getNorm(); // Distance in meters
            double lateralOffset = cameraToTarget.getY(); // Side-to-side alignment error

            // Adjust lateral offset based on left/right alignment strategy
            if (alignLeft) {
                lateralOffset += Constants.VisionConstants.Coral.leftOffsetMeters;
            } else {
                lateralOffset -= Constants.VisionConstants.Coral.rightOffsetMeters;
            }

            return Optional.of(new double[] { targetYaw, targetRange, lateralOffset, target.getFiducialId() });
        }

        return Optional.empty(); // No valid target found
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
