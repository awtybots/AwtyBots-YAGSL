// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
//import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.VecBuilder;
//import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private final SwerveDrive swerveDrive;
  private final SwerveDrivePoseEstimator poseEstimator;
  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);
  private final double headingBias = 0; // set this if there is alot of drift on pathplanner

  public SwerveSubsystem(File directory) {
    try {
      swerveDrive = new SwerveParser(directory)
          .createSwerveDrive(
              Constants.maxSpeed,
              new Pose2d(
                  new Translation2d(Meter.of(1), Meter.of(4)), Rotation2d.fromDegrees(0)));
      // Alternative method if you don't want to supply the conversion factor via JSON
      // files.
      // swerveDrive = new SwerveParser(directory).createSwerveDrive(maximumSpeed,
      // angleConversionFactor, driveConversionFactor);
    } catch (Exception e) {
      throw new RuntimeException(e);
    }

    setupPathPlanner();
    poseEstimator = new SwerveDrivePoseEstimator(
        getKinematics(),
        Rotation2d.fromDegrees(getGyroYaw()),
        swerveDrive.getModulePositions(),
        new Pose2d(0.0, 0.0, new Rotation2d()));
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void updateOdometry() {
    poseEstimator.update(
        gyro.getRotation2d(),
        swerveDrive.getModulePositions());

    boolean doRejectUpdate = false;
    boolean useMegaTag2 = true; // set to false to use MegaTag1
    // LimelightHelpers.SetRobotOrientation("",
    // poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0,
    // 0, 0);
    // LimelightHelpers.PoseEstimate mt2 =
    // LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");

    // // if our angular velocity is greater than 360 degrees per second, ignore
    // vision
    // // updates
    // if (Math.abs(gyro.getRate()) > 360) {
    // doRejectUpdate = true;
    // }
    // if (mt2.tagCount == 0) {
    // doRejectUpdate = true;
    // }
    // if (!doRejectUpdate) {
    // poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
    // poseEstimator.addVisionMeasurement(
    // mt2.pose,
    // mt2.timestampSeconds);
    if (useMegaTag2 == false) {
      LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");

      if (mt1.tagCount == 1 && mt1.rawFiducials.length == 1) {
        if (mt1.rawFiducials[0].ambiguity > .7) {
          doRejectUpdate = true;
        }
        if (mt1.rawFiducials[0].distToCamera > 3) {
          doRejectUpdate = true;
        }
      }
      if (mt1.tagCount == 0) {
        doRejectUpdate = true;
      }

      if (!doRejectUpdate) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5, .5, 9999999));
        poseEstimator.addVisionMeasurement(
            mt1.pose,
            mt1.timestampSeconds);
      }
    } else if (useMegaTag2 == true) {
      LimelightHelpers.SetRobotOrientation("", poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0,
          0, 0);
      LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("");
      if (Math.abs(gyro.getRate()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                          // vision updates
      {
        doRejectUpdate = true;
      }
      if (mt2.tagCount == 0) {
        doRejectUpdate = true;
      }
      if (!doRejectUpdate) {
        poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7, .7, 9999999));
        poseEstimator.addVisionMeasurement(
            mt2.pose,
            mt2.timestampSeconds);
      }
    }
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(
        () -> {
          swerveDrive.driveFieldOriented(velocity.get());
        });
  }

  /**
   * Resets odometry to the given pose. Gyro angle and module positions do not
   * need to be reset when calling this
   * method. However, if either gyro angle or module position is reset, this must
   * be called in order for odometry to
   * keep working.
   *
   * @param initialHolonomicPose The pose to set the odometry to
   */
  public void resetOdometry(Pose2d initialHolonomicPose) {

    double fieldOrientedOffset = 180.0;

    Rotation2d correctionHeading = Rotation2d.fromDegrees(getGyroYaw() - fieldOrientedOffset + headingBias);

    Pose2d biasedPose = new Pose2d(
        initialHolonomicPose.getTranslation(),
        correctionHeading);

    swerveDrive.resetOdometry(biasedPose);
    poseEstimator.resetPosition(
        correctionHeading,
        swerveDrive.getModulePositions(),
        biasedPose);
    System.out.println("Odometry Reset to: " + biasedPose);
  }

  public void setupPathPlanner() {
    // Load the RobotConfig from the GUI settings. You should probably
    // store this in your Constants file
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      // Configure AutoBuilder last
      AutoBuilder.configure(

          swerveDrive::getPose,
          // Robot pose supplier
          swerveDrive::resetOdometry,
          // Method to reset odometry (will be called if your auto has a starting pose)
          swerveDrive::getRobotVelocity,
          // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
          (speedsRobotRelative, moduleFeedForwards) -> {
            double flippedOmega = -speedsRobotRelative.omegaRadiansPerSecond;

            ChassisSpeeds correctedSpeeds = new ChassisSpeeds(
                speedsRobotRelative.vxMetersPerSecond,
                speedsRobotRelative.vyMetersPerSecond,
                flippedOmega);

            if (enableFeedforward) {
              swerveDrive.drive(
                  correctedSpeeds,
                  swerveDrive.kinematics.toSwerveModuleStates(correctedSpeeds),
                  moduleFeedForwards.linearForces());
            } else {
              swerveDrive.setChassisSpeeds(correctedSpeeds);
            }
          },
          // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also
          // optionally outputs individual module feedforwards
          new PPHolonomicDriveController(
              // PPHolonomicController is the built in path following controller for holonomic
              // drive trains
              new PIDConstants(1.2, 0.0, 0.0),
              // Translation PID constants
              new PIDConstants(0.9, 0.0, 0.0)
          // Rotation PID constants
          ),
          config,
          // The robot configuration
          () -> {
            // Boolean supplier that controls when the path will be mirrored for the red
            // alliance
            // This will flip the path being followed to the red side of the field.
            // THE ORIGIN WILL REMAIN ON THE BLUE SIDE

            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Red;
            }
            return false;
          },
          this
      // Reference to this subsystem to set requirements
      );

    } catch (Exception e) {
      // Handle exception as needed
      e.printStackTrace();
    }

    // Preload PathPlanner Path finding
    // IF USING CUSTOM PATHFINDER ADD BEFORE THIS LINE
    PathfindingCommand.warmupCommand().schedule();
  }

  /**
   * Get the path follower with events.
   *
   * @param pathName PathPlanner path name.
   * @return {@link AutoBuilder#followPath(PathPlannerPath)} path command.
   */
  public Command getAutonomousCommand(String TestAuto) {
    // Create a path following command using AutoBuilder. This will also trigger
    // event markers.
    return new PathPlannerAuto(TestAuto);
  }

 public Command driveToPose(Pose2d pose) {
    // Create the constraints to use while pathfinding
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 4.0,
        swerveDrive.getMaximumChassisAngularVelocity(), Units.degreesToRadians(720));

    // Since AutoBuilder is configured, we can use it to build pathfinding commands
    return AutoBuilder.pathfindToPose(
        pose,
        constraints,
        edu.wpi.first.units.Units.MetersPerSecond.of(0) // Goal end velocity in meters/sec
    );
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public void setInitialHeading(double angleDegrees) {
    gyro.setAngleAdjustment(angleDegrees);
    swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(angleDegrees)));
  }

  public void stop() {
    drive(new ChassisSpeeds(0, 0, 0));

    swerveDrive.setModuleStates(
        swerveDrive.kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0)), true);
  }

  /**
   * Get the swerve drive kinematics object.
   *
   * @return {@link SwerveDriveKinematics} of the swerve drive.
   */
  public SwerveDriveKinematics getKinematics() {
    return swerveDrive.kinematics;
  }

  public Pose2d getPose() {
    Pose2d rawPose = swerveDrive.getPose();
    Pose2d baisedPose = new Pose2d(
        rawPose.getTranslation(),
        rawPose.getRotation().plus(Rotation2d.fromDegrees(headingBias)));
    // return swerveDrive.getPose();
    return baisedPose;
  }

  public Command zeroHeadingCommand() {
    return this.runOnce(() -> gyro.reset());
  }

  public void zeroNavxGyroAuto() {
    gyro.reset();
    gyro.zeroYaw();
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative) {
    swerveDrive.drive(translation,
        rotation,
        fieldRelative,
        false); // Open loop is disabled since it shouldn't be used most of the time.
  }

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public double getGyroYaw() {
    return gyro.getYaw();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  @Override
  public void periodic() {
    poseEstimator.update(
        Rotation2d.fromDegrees(getGyroYaw()),
        swerveDrive.getModulePositions());

    LimelightHelpers.PoseEstimate limelightMeasurement = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
    if (limelightMeasurement.tagCount >= 2) { // Only trust measurement if we see multiple tags
      poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
      poseEstimator.addVisionMeasurement(
          limelightMeasurement.pose,
          limelightMeasurement.timestampSeconds);
      SmartDashboard.putNumber("Gyro Yaw", getGyroYaw());
      SmartDashboard.putNumber("Gyro Angle", getGyroAngle());

      Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
      SmartDashboard.putNumber("Odometry X", estimatedPose.getX());
      SmartDashboard.putNumber("Odometry Y", estimatedPose.getY());
      SmartDashboard.putNumber("Odometry Heading", estimatedPose.getRotation().getDegrees());
      SmartDashboard.putString("Odometry Pose: ", estimatedPose.toString());
      SmartDashboard.putString("PathPlanner Pose ", getPose().toString());

      var positions = swerveDrive.getModulePositions();
      for (int i = 0; i < 4; i++) {
        SmartDashboard.putNumber("module " + i, positions[i].angle.getDegrees());
      }

    }

  }
}
