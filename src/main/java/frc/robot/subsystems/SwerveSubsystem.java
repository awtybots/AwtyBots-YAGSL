// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;
import static edu.wpi.first.units.Units.Rotation;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DrivebaseConstants.TargetSide;
import frc.robot.Constants.VisionConstants;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import java.io.File;
import java.io.IOException;
import java.nio.file.Files;
import java.nio.file.Path;
import java.util.Optional;
import java.util.function.IntSupplier;
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
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
// Import the working team's Vision class:
import frc.robot.subsystems.Vision;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private final SwerveDrive swerveDrive;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final double headingBias = 0; // set this if there is alot of drift on pathplanner
  private final boolean visionDriveTest = VisionConstants.DRIVEWITHVISION;
  /**
   * PhotonVision class to keep an accurate odometry.
   */
  private Vision vision;

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

    // Always setup vision for alignment commands to work
    setupPhotonVision();

    // Only stop odometry thread if using vision for drive testing

    // swerveDrive.stopOdometryThread();

    setupPathPlanner();
  }

  public SwerveDrive getSwerveDrive() {
    return swerveDrive;
  }

  public void driveFieldOriented(ChassisSpeeds velocity) {
    swerveDrive.driveFieldOriented(velocity);
  }

  public Command driveFieldOriented(Supplier<ChassisSpeeds> velocity) {
    return run(() -> {
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
   * @param initialHolonomicPose The pose to set the odometry to.
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
            // System.out.println("Using Pose in AutoBuilder: " + currentPose);
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
   * Returns a path following command for autonomous routines.
   *
   * @param TestAuto PathPlanner path name.
   */
  public Command getAutonomousCommand(String TestAuto) {
    return new PathPlannerAuto(TestAuto);
  }

  public void drive(ChassisSpeeds velocity) {
    swerveDrive.drive(velocity);
  }

  public void addVisionMeasurement(Pose2d visionPose) {
    poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
  }

  public void setInitialHeading(double angleDegrees) {
    gyro.setAngleAdjustment(angleDegrees);
    swerveDrive.resetOdometry(new Pose2d(new Translation2d(0, 0), Rotation2d.fromDegrees(angleDegrees)));
  }

  public ChassisSpeeds getRobotVelocity() {
    return swerveDrive.getRobotVelocity();
  }

  private boolean fieldOriented = true; // Default to field-oriented mode

  public void setFieldOriented(boolean isFieldOriented) {
    this.fieldOriented = isFieldOriented;
    System.out.println("[SwerveSubsystem] Field-Oriented Mode: " + isFieldOriented);
  }

  public void drive(double forwardSpeed, double strafeSpeed, double rotationSpeed) {
    ChassisSpeeds speeds = new ChassisSpeeds(forwardSpeed, strafeSpeed, rotationSpeed);
    if (fieldOriented) {
      // Convert to field-relative speeds if needed
      speeds = ChassisSpeeds.fromFieldRelativeSpeeds(forwardSpeed, strafeSpeed, rotationSpeed, getPose().getRotation());
    }
    swerveDrive.drive(speeds);
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
    swerveDrive.resetOdometry(newPose);
    poseEstimator.resetPosition(
        newPose.getRotation(),
        swerveDrive.getModulePositions(),
        newPose);

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
  private Pose2d lastLoggedPose = new Pose2d();

  @Override
  public void periodic() {

    poseEstimator.update(
        Rotation2d.fromDegrees(getGyroYaw()),
        swerveDrive.getModulePositions());
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();

    if (loopCounter % 10 == 0) {

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

    // --- Vision Integration in periodic() ---
    vision.updatePoseEstimation(swerveDrive);
    // swerveDrive.updateOdometry();
    Pose2d fused = swerveDrive.getPose();

    int currAprilTagTarget = vision.getBestReefTarget();
    SmartDashboard.putNumber("Vision/AprilTag", currAprilTagTarget);
    SmartDashboard.putData("Field", swerveDrive.field);
    if (!fused.equals(lastLoggedPose)) {
      System.out.printf("[SwerveSubsystem] Odometry Pose: %s%n", fused);
      lastLoggedPose = fused;
    }
  }

  @Override
  public void simulationPeriodic() {

    poseEstimator.update(
        Rotation2d.fromDegrees(getGyroYaw()),
        swerveDrive.getModulePositions());
    Pose2d estimatedPose = poseEstimator.getEstimatedPosition();

    if (loopCounter % 10 == 0) {

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

    // --- Vision Integration in periodic() ---
    vision.updatePoseEstimation(swerveDrive);
    // swerveDrive.updateOdometry();
    Pose2d fused = swerveDrive.getPose();

    int currAprilTagTarget = vision.getBestReefTarget();
    SmartDashboard.putNumber("Vision/AprilTag", currAprilTagTarget);
    SmartDashboard.putData("Field", swerveDrive.field);
    if (!fused.equals(lastLoggedPose)) {
      System.out.printf("[SwerveSubsystem] Odometry Pose: %s%n", fused);
      lastLoggedPose = fused;
    }
  }

  // --- Vision Setup and Commands ---

  // Instantiate the Vision class.
  public void setupPhotonVision() {
    vision = new Vision(swerveDrive::getPose, swerveDrive.field);
  }

  public int getReefTargetTagID() {
    return (vision.getBestReefTarget());
  }

  // Command that uses AutoBuilder's pathfinding to drive to a specified pose.
  public Command driveToPose(Pose2d pose) {
    System.out.println("[SwerveSubsystem] driveToPose called with pose: " + pose);
    PathConstraints constraints = new PathConstraints(
        swerveDrive.getMaximumChassisVelocity(), 2.5,
        swerveDrive.getMaximumChassisAngularVelocity(), Math.toRadians(720));
    return AutoBuilder.pathfindToPose(pose, constraints, edu.wpi.first.units.Units.MetersPerSecond.of(0));
  }

  // Command to align the robot for reef scoring using a detected AprilTag ID and
  // target side.
  // Command to align the robot for reef scoring using a detected AprilTag ID and
  // target side.
  public Command alignToReefScore(int aprilTag, Constants.DrivebaseConstants.TargetSide scoringSide) {
    return Commands.deferredProxy(() -> {
      System.out.println(
          "[SwerveSubsystem] Starting reef alignment. AprilTag: " + aprilTag + ", Scoring Side: " + scoringSide);

      // Check if vision system is available
      if (vision == null) {
        System.out.println("[SwerveSubsystem] ERROR: Vision system not initialized!");
        return Commands.none();
      }

      // Use current target if aprilTag is 0 or invalid
      int targetTag = aprilTag;
      if (aprilTag <= 0) {
        targetTag = vision.getBestReefTarget();
        System.out.println("[SwerveSubsystem] Using best reef target: " + targetTag);
      }

      // Validate target
      if (targetTag <= 0 || !vision.isValidTargetForScoring(targetTag)) {
        System.out.println("[SwerveSubsystem] No valid AprilTag target for reef alignment: " + targetTag);
        return Commands.none();
      }

      // Calculate robot offset based on scoring side
      Transform2d robotOffset;
      if (scoringSide == Constants.DrivebaseConstants.TargetSide.LEFT) {
        robotOffset = new Transform2d(
            new Translation2d(Constants.DrivebaseConstants.ReefXDistance, Constants.DrivebaseConstants.ReefLeftYOffset),
            Rotation2d.fromDegrees(180));
        System.out.println("[SwerveSubsystem] Using LEFT robot offset: " + robotOffset);
      } else {
        robotOffset = new Transform2d(
            new Translation2d(Constants.DrivebaseConstants.ReefXDistance,
                Constants.DrivebaseConstants.ReefRightYOffset),
            Rotation2d.fromDegrees(180));
        System.out.println("[SwerveSubsystem] Using RIGHT robot offset: " + robotOffset);
      }

      try {
        Pose2d targetPose = Vision.getAprilTagPose(targetTag, robotOffset);
        System.out.println("[SwerveSubsystem] Calculated target pose: " + targetPose);
        System.out.println("[SwerveSubsystem] Current pose: " + getPose());

        return driveToPose(targetPose);
      } catch (Exception e) {
        System.out.println("[SwerveSubsystem] ERROR calculating target pose: " + e.getMessage());
        return Commands.none();
      }
    });
  }

  public Command alignToReefScore(IntSupplier aprilTagSupplier, TargetSide scoringSide) {
    return alignToReefScore(aprilTagSupplier.getAsInt(), scoringSide);
  }

  public void debugVisionSystem() {
    if (vision == null) {
      System.out.println("[DEBUG] Vision system is NULL!");
      return;
    }

    int reefTarget = vision.getBestReefTarget();
    System.out.println("[DEBUG] Best reef target: " + reefTarget);

    if (reefTarget > 0) {
      boolean isValid = vision.isValidTargetForScoring(reefTarget);
      System.out.println("[DEBUG] Target " + reefTarget + " is valid: " + isValid);

      double distance = vision.getDistanceFromAprilTag(reefTarget);
      System.out.println("[DEBUG] Distance to target: " + distance);
    }

    // Check camera connectivity
    for (Vision.Cameras camera : Vision.Cameras.values()) {
      boolean connected = camera.camera.isConnected();
      System.out.println("[DEBUG] Camera " + camera.name() + " connected: " + connected);
    }
  }
}
