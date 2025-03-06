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

    private double smoothedDistance = 0.0;
    private final double SMOOTHING_FACTOR = 0.8;

    public void logVisionData() {
        var targetOpt = getBestTarget();

        if (targetOpt.isPresent()) {
            PhotonTrackedTarget target = targetOpt.get();
            Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(target.getFiducialId());

            if (tagPoseOpt.isPresent()) {
                Transform3d cameraToTarget = target.getBestCameraToTarget(); // Use PhotonVision's 3D transform

                double rawDistance = cameraToTarget.getTranslation().getNorm(); // Accurate distance
                smoothedDistance = (SMOOTHING_FACTOR * rawDistance) + ((1 - SMOOTHING_FACTOR) * smoothedDistance);

                SmartDashboard.putBoolean("Vision/AprilTag Found", true);
                SmartDashboard.putNumber("Vision/AprilTag ID", target.getFiducialId());
                SmartDashboard.putNumber("Vision/Yaw (degrees)", target.getYaw());
                SmartDashboard.putNumber("Vision/Distance (m)", smoothedDistance); // Use smoothed distance!
                SmartDashboard.putNumber("Vision/Lateral Offset (m)", cameraToTarget.getY());
            }
        } else {
            SmartDashboard.putBoolean("Vision/AprilTag Found", false);
        }
    }

    @Override
    public void periodic() {
        logVisionData();
    }

    @Override
    public void simulationPeriodic() {
        logVisionData();
    }
}
