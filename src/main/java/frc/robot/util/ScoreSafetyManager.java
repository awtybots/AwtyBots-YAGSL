package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Constants;

/**
 * Tracks when the robot must back away from the reef before allowing arm/elevator motion.
 */
public final class ScoreSafetyManager {

  private static boolean lockoutActive = false;
  private static Pose2d lockPose = new Pose2d();
  private static double lastReminderTimestamp = 0.0;

  private ScoreSafetyManager() {
    // utility class
  }

  /**
   * Arm/elevator moves are blocked until the chassis has moved away from the recorded pose.
   */
  public static void activateLockout(Pose2d poseAtLockout) {
    if (!DriverStation.isTeleop()) {
      // Skip locking out during auto; feature is intended for teleop reef scoring.
      lockoutActive = false;
      return;
    }
    lockoutActive = true;
    lockPose = poseAtLockout;
  }

  public static void clearLockout() {
    lockoutActive = false;
  }

  /**
   * Called periodically with the current robot pose so the lockout can clear itself.
   */
  public static void updateWithCurrentPose(Pose2d currentPose) {
    if (!lockoutActive) {
      return;
    }

    Translation2d currentTranslation = currentPose.getTranslation();
    double distance = currentTranslation.getDistance(lockPose.getTranslation());
    if (distance >= Constants.SafetyConstants.BACKUP_RELEASE_DISTANCE_METERS) {
      lockoutActive = false;
    }
  }

  public static boolean isMovementBlocked() {
    return lockoutActive;
  }

  /**
   * Throttled operator reminder that the arm/elevator are intentionally locked.
   */
  public static void notifyBlocked() {
    if (!lockoutActive) {
      return;
    }

    double now = Timer.getFPGATimestamp();
    if (now - lastReminderTimestamp >= Constants.SafetyConstants.LOCK_REMINDER_INTERVAL_SECONDS) {
      DriverStation.reportWarning("Back up before moving the arm/elevator", false);
      lastReminderTimestamp = now;
    }
  }
}

