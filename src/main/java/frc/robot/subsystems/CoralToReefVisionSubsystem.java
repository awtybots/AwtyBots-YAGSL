package frc.robot.subsystems;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import frc.robot.Constants;
import frc.robot.subsystems.SwerveSubsystem;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

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
        this.robotToCameraTransforms = cameraTransforms;
        this.fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeAndyMark);

        for (int i = 0; i < cameraNames.size(); i++) {
            PhotonCamera camera = new PhotonCamera(cameraNames.get(i));
            cameras.add(camera);

            // FIX: Create PhotonPoseEstimator correctly using the new constructor
            PhotonPoseEstimator estimator = new PhotonPoseEstimator(
                    fieldLayout,
                    PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
                    cameraTransforms.get(i) // Use Transform3d correctly
            );
            estimator.setMultiTagFallbackStrategy(PoseStrategy.LOWEST_AMBIGUITY);
            photonPoseEstimators.add(estimator);
        }
    }

    /**
     * Estimates the robot's field pose using AprilTags from **all unread results**.
     */
    public Optional<Pose2d> getEstimatedFieldPose() {
        for (int i = 0; i < cameras.size(); i++) {
            List<PhotonPipelineResult> results = cameras.get(i).getAllUnreadResults();
            for (PhotonPipelineResult result : results) {
                if (result.hasTargets()) {
                    Transform3d cameraToTarget = result.getBestTarget().getBestCameraToTarget();
                    Pose3d estimatedPose3d = new Pose3d().transformBy(robotToCameraTransforms.get(i))
                            .transformBy(cameraToTarget);

                    System.out.println("[Vision] Raw Estimated Pose: " + estimatedPose3d.toPose2d());
                    return Optional.of(estimatedPose3d.toPose2d());
                }
            }
        }
        return Optional.empty();
    }

    /**
     * Determines the best reef pose to align with.
     */
    public Pose2d getBestReefPos(boolean alignLeft) {
        // Require vision pose for alignment
        Optional<Pose2d> visionPoseOpt = getEstimatedFieldPose();
        if (visionPoseOpt.isEmpty()) {
            System.out.println("[Vision] No valid target detected! Stopping.");
            return null; // Stop if no vision data
        }

        Pose2d robotPose = visionPoseOpt.get(); // Always use vision-based pose
        Pose2d bestPose = new Pose2d();
        double bestDistance = Double.MAX_VALUE;

        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

        for (Pose2d[] poses : (alliance == DriverStation.Alliance.Red)
                ? Constants.VisionConstants.Coral.redReefScoringPoses.values()
                : Constants.VisionConstants.Coral.blueReefScoringPoses.values()) {

            // Choose left or right based on alignment preference
            Pose2d candidate = alignLeft ? poses[0] : poses[1];
            double distance = candidate.getTranslation().getDistance(robotPose.getTranslation());
            if (distance < bestDistance) {
                bestDistance = distance;
                bestPose = candidate;
            }
        }

        // Log for debugging
        System.out.println("[Vision] Vision Pose: " + robotPose);
        System.out.println("[Vision] Selected Target Pose: " + bestPose);

        return bestPose;
    }

    /**
     * Updates odometry with vision-based pose estimation.
     */
    public void updateOdometryWithVision() {
        Optional<Pose2d> estimatedPoseOpt = getEstimatedFieldPose();
        estimatedPoseOpt.ifPresent(swerve::addVisionMeasurement);
    }

    @Override
    public void periodic() {
        updateOdometryWithVision();
    }
}
