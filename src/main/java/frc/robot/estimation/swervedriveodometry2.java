package frc.robot.estimation;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class swervedriveodometry2 extends SwerveDrivePoseEstimator {

  private final int numModules;
  private SwerveModulePosition[] previousPositions;
  private Rotation2d previousGyro;
  private double lastTime = Double.NaN;

  public swervedriveodometry2(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Translation2d[] moduleLocations,
      Pose2d initialPoseMeters) {
    this(
        kinematics,
        gyroAngle,
        modulePositions,
        initialPoseMeters,
        VecBuilder.fill(0.1, 0.1, 0.1),
        VecBuilder.fill(0.9, 0.9, 0.9));
  }

  public swervedriveodometry2(
      SwerveDriveKinematics kinematics,
      Rotation2d gyroAngle,
      SwerveModulePosition[] modulePositions,
      Pose2d initialPoseMeters,
      Matrix<N3, N1> stateStdDevs,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super(kinematics, gyroAngle, modulePositions, initialPoseMeters, stateStdDevs, visionMeasurementStdDevs);

    numModules = modulePositions.length;
    previousPositions = copyPositions(modulePositions);
    previousGyro = gyroAngle;

    SmartDashboard.putString("Odo2/Status", "Initialized (simple collection)");
  }

  @Override
  public Pose2d updateWithTime(double currentTimeSeconds, Rotation2d gyroAngle, SwerveModulePosition[] wheelPositions) {
    if (wheelPositions.length != numModules) {
      throw new IllegalArgumentException(
          "Number of modules is not consistent with number of wheel locations provided in constructor");
    }

    // Compute dt
    double dt;
    if (Double.isNaN(lastTime)) {
      dt = 0.02;
    } else {
      dt = Math.max(1e-6, currentTimeSeconds - lastTime);
    }
    lastTime = currentTimeSeconds;
    SmartDashboard.putNumber("Odo2/dt", dt);

    // --- Previous and current gyro ---
    double theta0 = previousGyro.getRadians();
    double theta1 = gyroAngle.getRadians();
    SmartDashboard.putNumber("Odo2/PrevGyro_rad", theta0);
    SmartDashboard.putNumber("Odo2/CurrGyro_rad", theta1);

    // --- Drive distance and steering of each module ---
    for (int i = 0; i < numModules; i++) {
      double prevDistance = previousPositions[i].distanceMeters;
      double currDistance = wheelPositions[i].distanceMeters;

      double prevAngle = previousPositions[i].angle.getRadians();
      double currAngle = wheelPositions[i].angle.getRadians();

      SmartDashboard.putNumber("Odo2/Module" + i + "/PrevDistance_m", prevDistance);
      SmartDashboard.putNumber("Odo2/Module" + i + "/CurrDistance_m", currDistance);
      SmartDashboard.putNumber("Odo2/Module" + i + "/PrevAngle_rad", prevAngle);
      SmartDashboard.putNumber("Odo2/Module" + i + "/CurrAngle_rad", currAngle);
    }

    // Update history
    previousPositions = copyPositions(wheelPositions);
    previousGyro = gyroAngle;

    SmartDashboard.putString("Odo2/Status", "OK (simple collection)");

    // Does not modify pose, just returns the current estimate
    return super.getEstimatedPosition();
  }

  // ---------- Utilities ----------
  private SwerveModulePosition[] copyPositions(SwerveModulePosition[] original) {
    SwerveModulePosition[] copy = new SwerveModulePosition[original.length];
    for (int i = 0; i < original.length; i++) {
      copy[i] = new SwerveModulePosition(original[i].distanceMeters, original[i].angle);
    }
    return copy;
  }
}
