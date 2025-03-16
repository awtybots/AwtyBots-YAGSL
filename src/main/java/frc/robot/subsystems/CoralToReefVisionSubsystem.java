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

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;
import java.util.Optional;

public class CoralToReefVisionSubsystem extends SubsystemBase {
    private final SwerveSubsystem swerve;
    private final List<PhotonCamera> cameras = new ArrayList<>();
    private final AprilTagFieldLayout fieldLayout;
    private final List<PhotonPoseEstimator> photonPoseEstimators = new ArrayList<>();
    private final List<Transform3d> robotToCameraTransforms;

    private Optional<Pose2d> lastFieldPose = Optional.empty();
    private long lastUpdateTimeMs = 0;

    public CoralToReefVisionSubsystem(
            SwerveSubsystem swerve,
            List<String> cameraNames,
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
        Optional<Pose2d> bestPoseOpt = Optional.empty();
        int maxTagsUsed = 0; // ✅ Track the best pose based on tag count

        for (int i = 0; i < cameras.size(); i++) {
            List<PhotonPipelineResult> results = cameras.get(i).getAllUnreadResults();
            for (PhotonPipelineResult result : results) {
                var poseResultOpt = photonPoseEstimators.get(i).update(
                        result,
                        cameras.get(i).getCameraMatrix(),
                        cameras.get(i).getDistCoeffs());

                if (poseResultOpt.isPresent()) {
                    var poseResult = poseResultOpt.get();
                    Pose2d estimatedPose = poseResult.estimatedPose.toPose2d();
                    int tagsUsed = poseResult.targetsUsed.size(); // ✅ Use tag count

                    // ✅ Choose the pose with the highest tag count
                    if (tagsUsed > maxTagsUsed) {
                        maxTagsUsed = tagsUsed;
                        bestPoseOpt = Optional.of(estimatedPose);
                    }

                }
            }
        }

        // ✅ Update the last valid pose if a better pose was found
        if (bestPoseOpt.isPresent()) {
            lastFieldPose = bestPoseOpt;
            lastUpdateTimeMs = System.currentTimeMillis();
            System.out.println("[Vision] Estimated Field Pose: " + bestPoseOpt);
            return bestPoseOpt;
        }

        return Optional.empty();
    }

    public Pose2d getBestReefPos(boolean alignLeft) {
        // 1) Grab the current robot position from your swerve subsystem
        Pose2d robotPose = swerve.getPose();

        // 2) Pick which array of reef poses to loop over
        DriverStation.Alliance alliance = DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);
        Collection<Pose2d[]> possiblePosesCollections = (alliance == DriverStation.Alliance.Red)
                ? Constants.VisionConstants.Coral.redReefScoringPoses.values()
                : Constants.VisionConstants.Coral.blueReefScoringPoses.values();

        // 3) Among all possible reef-scoring positions, pick the one that’s closest
        Pose2d bestPose = null;
        double bestDistance = Double.MAX_VALUE;
        for (Pose2d[] pair : possiblePosesCollections) {
            // This pair is [ leftBarPose, rightBarPose ]
            Pose2d candidate = alignLeft ? pair[0] : pair[1];
            double distance = candidate.getTranslation().getDistance(robotPose.getTranslation());
            if (distance < bestDistance) {
                bestDistance = distance;
                bestPose = candidate;
            }
        }

        if (bestPose == null) {
            // Return an empty pose if no valid tags found
            return new Pose2d();
        }

        return bestPose;
    }

    /**
     * Determines the best reef pose to align with.
     */
    // public Pose2d getBestReefPos(boolean alignLeft) {
    // // Require vision pose for alignment
    // Optional<Pose2d> visionPoseOpt = getEstimatedFieldPose();
    // if (visionPoseOpt.isEmpty()) {
    // System.out.println("[Vision] No valid target detected! Stopping.");
    // return null; // Stop if no vision data
    // }

    // Pose2d robotPose = visionPoseOpt.get(); // Always use vision-based pose
    // Pose2d bestPose = new Pose2d();
    // double bestDistance = Double.MAX_VALUE;

    // DriverStation.Alliance alliance =
    // DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);

    // for (Pose2d[] poses : (alliance == DriverStation.Alliance.Red)
    // ? Constants.VisionConstants.Coral.redReefScoringPoses.values()
    // : Constants.VisionConstants.Coral.blueReefScoringPoses.values()) {

    // // Choose left or right based on alignment preference
    // Pose2d candidate = alignLeft ? poses[0] : poses[1];
    // double distance =
    // candidate.getTranslation().getDistance(robotPose.getTranslation());
    // if (distance < bestDistance) {
    // bestDistance = distance;
    // bestPose = candidate;
    // }
    // }

    // // Log for debugging
    // System.out.println("[Vision] Vision Pose: " + robotPose);
    // System.out.println("[Vision] Selected Target Pose: " + bestPose);

    // return bestPose;
    // }

    /**
     * Returns the last valid field pose (as stored by getEstimatedFieldPose()).
     * This is used by the command so it isn’t forced to receive an empty Optional
     * when no new unread results are available.
     */
    public Optional<Pose2d> getTargetPose() {
        return lastFieldPose;
    }

    /**
     * Returns the first detected AprilTag ID from all unread results.
     */

    private Optional<Integer> lastDetectedTag = Optional.empty();

    public Optional<Integer> getDetectedTagID() {
        // Look for new results.
        for (int i = 0; i < cameras.size(); i++) {
            List<PhotonPipelineResult> results = cameras.get(i).getAllUnreadResults();
            for (PhotonPipelineResult result : results) {
                if (result.hasTargets()) {
                    // Retrieve the best target’s fiducial ID and cache it.
                    PhotonTrackedTarget bestTarget = result.getBestTarget();
                    lastDetectedTag = Optional.of(bestTarget.getFiducialId());
                    return lastDetectedTag;
                }
            }
        }
        // Return the cached value if no new result is found.
        return lastDetectedTag;
    }

    /**
     * Updates odometry with vision-based pose estimation.
     */
    public void updateOdometryWithVision() {
        Optional<Pose2d> estimatedPoseOpt = getEstimatedFieldPose();
        // When a new vision measurement is available, immediately reset odometry.
        estimatedPoseOpt.ifPresent(visionPose -> {
            swerve.updateOdometry(visionPose);
        });
    }

    @Override
    public void periodic() {
        updateOdometryWithVision();
    }
}
