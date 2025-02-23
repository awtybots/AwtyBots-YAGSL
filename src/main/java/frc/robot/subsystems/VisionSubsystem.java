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
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;

public class VisionSubsystem extends SubsystemBase {
    private final PhotonCamera camera = new PhotonCamera(Constants.VisionConstants.limelightAprilTagCamera);
    private SwerveSubsystem swerve;

    private final AprilTagFieldLayout fieldLayout; // AprilTag field layout
    private final Transform3d robotToCamera; // Camera position relative to the robot
    private final PhotonPoseEstimator photonPoseEstimator; // Vision-based pose estimator

    public VisionSubsystem(SwerveSubsystem swerve) {
        this.swerve = swerve;

        // Load the AprilTag field layout (2025 game)
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

        // Define the camera's position relative to the robot (adjust values based on
        // actual setup)
        robotToCamera = new Transform3d(0.2, 0.0, 0.5, new Rotation3d(0, 0, 0));

        // Initialize the pose estimator using the camera and tag layout
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.LOWEST_AMBIGUITY, robotToCamera);

    }

    public Optional<PhotonTrackedTarget> getBestTarget() {
        PhotonPipelineResult result = camera.getLatestResult();
        if (result.hasTargets()) {
            return Optional.of(result.getBestTarget());
        }
        return Optional.empty();
    }

    public void logAprilTagData() {
        getBestTarget().ifPresentOrElse(target -> {
            SmartDashboard.putBoolean("AprilTag Found", true);
            SmartDashboard.putNumber("AprilTag ID", target.getFiducialId());
            SmartDashboard.putNumber("AprilTag Yaw", target.getYaw());
            SmartDashboard.putNumber("AprilTag Pitch", target.getPitch());
            SmartDashboard.putNumber("AprilTag Skew", target.getSkew());
            SmartDashboard.putNumber("AprilTag Area", target.getArea());
        }, () -> {
            SmartDashboard.putBoolean("AprilTag Found", false);
        });
    }

    public Optional<double[]> getAlignmentErrors() {
        return getBestTarget().map(target -> {
            double yawError = target.getYaw(); // Angle error (rotation)

            // Compute Distance to Target
            double targetRange = PhotonUtils.calculateDistanceToTargetMeters(
                    0.5, // Camera height in meters (adjust for your robot)
                    1.435, // AprilTag height in meters (2024 field values)
                    Units.degreesToRadians(-30.0), // Camera mount angle (adjust based on actual)
                    Units.degreesToRadians(target.getPitch()));

            // Distance error (difference from desired target distance)
            double distanceError = targetRange - Constants.VisionConstants.targetDistanceMeters;

            // Left/Right Offset Adjustment
            double lateralOffset = Constants.VisionConstants.leftOffsetMeters
                    - Constants.VisionConstants.rightOffsetMeters;

            return new double[] { yawError, distanceError, lateralOffset };
        });
    }

    public void updatePoseEstimation() {
        PhotonPipelineResult result = camera.getLatestResult();
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