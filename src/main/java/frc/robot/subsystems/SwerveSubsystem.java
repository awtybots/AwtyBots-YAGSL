// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS.NavXComType;

import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private final SwerveDrive swerveDrive;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final double headingBias = -2; // set this if there is alot of drift on pathplanner

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

    poseEstimator = new SwerveDrivePoseEstimator(
        getKinematics(),
        Rotation2d.fromDegrees(getGyroYaw()),
        swerveDrive.getModulePositions(),
        new Pose2d(0.0, 0.0, new Rotation2d()));

    setupPathPlanner();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
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

  private AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

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
          () -> {
            Pose2d currentPose = getPose();
            System.out.println("Using Pose in AutoBuilder: " + currentPose);
            return currentPose;
          },
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

  public PathConstraints getPathConstraintsFromSettings() {
    Path path = Path.of(Filesystem.getDeployDirectory().toString(), "pathplanner/settings.json");
    ObjectMapper objectMapper = new ObjectMapper();

    try {
      JsonNode rootNode = objectMapper.readTree(Files.readAllBytes(path));

      double maxVelocity = rootNode.get("defaultMaxVel").asDouble(2.0); // Default: 2.0 m/s
      double maxAcceleration = rootNode.get("defaultMaxAccel").asDouble(3.0); // Default: 3.0 m/s²
      double maxAngularVelocity = Math.toRadians(rootNode.get("defaultMaxAngVel").asDouble(180.0)); // Convert degrees
                                                                                                    // to radians
      double maxAngularAcceleration = Math.toRadians(rootNode.get("defaultMaxAngAccel").asDouble(360.0)); // Convert
                                                                                                          // degrees to
                                                                                                          // radians

      return new PathConstraints(maxVelocity, maxAcceleration, maxAngularVelocity, maxAngularAcceleration);
    } catch (IOException e) {
      System.out.println("[ERROR] Failed to read PathPlanner settings.json: " + e.getMessage());
      return new PathConstraints(2.0, 3.0, Math.toRadians(180.0), Math.toRadians(360.0)); // Fallback defaults
    }
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

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public void setInitialHeading(double angleDegrees) {
    gyro.setAngleAdjustment(angleDegrees);
    swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(angleDegrees)));
  }

  public void setInitialHeadingWithVision(double targetAngleDegrees, CoralToReefVisionSubsystem vision) {
    var alliance = DriverStation.getAlliance();

    // Flip target heading for Red Alliance
    if (alliance.isPresent() && alliance.get() == Alliance.Red) {
      targetAngleDegrees = -targetAngleDegrees;
    }

    // Get estimated heading from vision
    Optional<Pose2d> visionPoseOpt = vision.getEstimatedPose();
    if (visionPoseOpt.isPresent()) {
      double estimatedFieldHeading = visionPoseOpt.get().getRotation().getDegrees();
      double gyroCurrentHeading = getGyroYaw(); // Get current gyro heading

      // Calculate correction offset
      double headingCorrection = estimatedFieldHeading - gyroCurrentHeading;

      // Apply correction
      gyro.setAngleAdjustment(gyroCurrentHeading + headingCorrection);
      System.out.println("[GyroReset] Adjusted with Vision Offset: " + headingCorrection + " degrees");
    } else {
      System.out.println("[GyroReset] No vision data available, setting based on gyro alone.");
      gyro.setAngleAdjustment(targetAngleDegrees);
    }

    swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(targetAngleDegrees)));
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

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public void updateOdometry(Pose2d newPose) {
    resetOdometry(newPose);
    System.out.println("[SwerveSubsystem] Updated PathPlanner Start Position: " + newPose);
  }

  public void addVisionMeasurement(Pose2d visionPose, double timestamp) {
    poseEstimator.addVisionMeasurement(visionPose, timestamp);
  }

  public double getGyroYaw() {
    return gyro.getYaw();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  private int loopCounter = 0;

  @Override
  public void periodic() {
    poseEstimator.update(
        Rotation2d.fromDegrees(getGyroYaw()),
        swerveDrive.getModulePositions());

    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();
    if (loopCounter % 10 == 0) { // Run every 10 cycle, updates every ~200 ms
      SmartDashboard.putNumber("Gyro Yaw", getGyroYaw());
      SmartDashboard.putNumber("Gyro Angle", getGyroAngle());
      SmartDashboard.putNumber("Odometry X", estimatedPose.getX());
      SmartDashboard.putNumber("Odometry Y", estimatedPose.getY());
      SmartDashboard.putNumber("Odometry Heading", estimatedPose.getRotation().getDegrees());
      SmartDashboard.putString("Odometry Pose: ", estimatedPose.toString());
      SmartDashboard.putString("PathPlanner Pose ", getPose().toString());
    }
    loopCounter++;

    var positions = swerveDrive.getModulePositions();
    for (int i = 0; i < 4; i++) {
      SmartDashboard.putNumber("module " + i, positions[i].angle.getDegrees());
    }

  }

}
