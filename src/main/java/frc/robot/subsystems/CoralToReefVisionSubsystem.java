package frc.robot.subsystems;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.subsystems.SwerveSubsystem;
import java.util.List;
import java.util.Optional;
import java.util.ArrayList;

public class CoralToReefVisionSubsystem extends SubsystemBase {
    private final SwerveSubsystem swerve;
    private final List<PhotonCamera> cameras = new ArrayList<>();
    private final AprilTagFieldLayout fieldLayout;
    private final List<PhotonPoseEstimator> photonPoseEstimators = new ArrayList<>();
    private final List<Transform3d> robotToCameraTransforms;

    public CoralToReefVisionSubsystem(SwerveSubsystem swerve, List<String> cameraNames,
            List<Transform3d> cameraTransforms) {
        this.swerve = swerve;
        fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);
        this.robotToCameraTransforms = cameraTransforms;

        for (int i = 0; i < cameraNames.size(); i++) {
            PhotonCamera camera = new PhotonCamera(cameraNames.get(i));
            cameras.add(camera);
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR, cameraTransforms.get(i));
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            photonPoseEstimators.add(estimator);
        }
    }

    public Optional<Pair<Integer, Pose2d>> getEstimatedFieldPose() {
        for (int i = 0; i < cameras.size(); i++) {
            PhotonPipelineResult result = cameras.get(i).getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget bestTarget = result.getBestTarget();
                int aprilTagID = bestTarget.getFiducialId();

                // Use predefined AprilTag pose if available
                Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(aprilTagID);
                if (tagPoseOpt.isPresent()) {
                    Pose2d tagPose = tagPoseOpt.get().toPose2d();
                    return Optional.of(Pair.of(aprilTagID, tagPose));
                }
            }
        }
        return Optional.empty();
    }

    public void updateOdometryWithVision() {
        Optional<Pair<Integer, Pose2d>> estimatedPoseOpt = getEstimatedFieldPose();
        if (estimatedPoseOpt.isPresent()) {
            Pose2d estimatedPose = estimatedPoseOpt.get().getSecond(); // Extract Pose2d
            swerve.addVisionMeasurement(estimatedPose);
        }
    }

    public void logVisionData() {
        Optional<Pair<Integer, Pose2d>> estimatedPoseOpt = getEstimatedFieldPose();
        if (estimatedPoseOpt.isPresent()) {
            int aprilTagID = estimatedPoseOpt.get().getFirst(); // Extract AprilTag ID
            Pose2d estimatedPose = estimatedPoseOpt.get().getSecond(); // Extract Pose2d

            SmartDashboard.putBoolean("Vision/01 Valid Target", true);
            SmartDashboard.putNumber("Vision/02 AprilTag ID", aprilTagID); // Log detected AprilTag ID
            SmartDashboard.putNumber("Vision/03 Estimated X", estimatedPose.getX());
            SmartDashboard.putNumber("Vision/04 Estimated Y", estimatedPose.getY());
            SmartDashboard.putNumber("Vision/05 Estimated Rotation", estimatedPose.getRotation().getDegrees());
        } else {
            SmartDashboard.putBoolean("Vision/01 Valid Target", false);
        }
    }

    @Override
    public void periodic() {
        updateOdometryWithVision();
        logVisionData();
    }
}
