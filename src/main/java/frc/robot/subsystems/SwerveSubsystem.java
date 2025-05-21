// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.Meter;

import java.io.File;
import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.commands.PathfindingCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.numbers.N2;
import edu.wpi.first.math.system.LinearSystem;
import edu.wpi.first.math.system.plant.LinearSystemId;
import java.util.Random;
import edu.wpi.first.math.system.plant.struct.DCMotorStruct;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.Robot;

public class SwerveSubsystem extends SubsystemBase {

  File directory = new File(Filesystem.getDeployDirectory(), "swerve");
  private final SwerveDrive swerveDrive;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final double headingBias = 0; // set this if there is alot of drift on pathplanner

  // Simulation-specific fields
  private Random random = new Random();
  private LinearSystem<N2, N2, N2> drivetrainSystem;
  private double lastSimTime;

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
        Rotation2d.fromDegrees(getGyroAngle()),
        swerveDrive.getModulePositions(),
        new Pose2d(7.0, 4.0, new Rotation2d()));

    // Initialize simulation system
    if (Robot.isSimulation()) {
      setupSimulation();
    }
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

    double fieldOrientedOffset = 0;

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
    RobotConfig config;
    try {
      config = RobotConfig.fromGUISettings();

      final boolean enableFeedforward = true;
      
      AutoBuilder.configure(
          this::getPose,
          swerveDrive::resetOdometry,
          swerveDrive::getRobotVelocity,
          (speedsRobotRelative, moduleFeedForwards) -> {
            // Correct for drift and apply feedforward
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
          new PPHolonomicDriveController(
              new PIDConstants(Constants.DriveConstants.kPTranslation, 
                             Constants.DriveConstants.kITranslation,
                             Constants.DriveConstants.kDTranslation),
              new PIDConstants(Constants.DriveConstants.kPRotation,
                             Constants.DriveConstants.kIRotation,
                             Constants.DriveConstants.kDRotation)
          ),
          config,
          () -> {
            var alliance = DriverStation.getAlliance();
            if (alliance.isPresent()) {
              return alliance.get() == DriverStation.Alliance.Blue;
            }
            return false;
          },
          this
      );

    } catch (Exception e) {
      e.printStackTrace();
    }

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

  public void zeroGyro() {
    swerveDrive.zeroGyro();
  }

  public double getGyroYaw() {
    return gyro.getYaw();
  }

  public double getGyroAngle() {
    return gyro.getAngle();
  }

  SwerveModuleState[] states = new SwerveModuleState[] {
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState(),
      new SwerveModuleState()
  };

  StructArrayPublisher<SwerveModuleState> publisher = NetworkTableInstance.getDefault()
      .getStructArrayTopic("SpecialChild", SwerveModuleState.struct).publish();

  private void updateTelemetry() {
    // Get current states and positions
    SwerveModuleState[] states = swerveDrive.getStates();
    var modulePositions = swerveDrive.getModulePositions();
    var chassisSpeeds = swerveDrive.getRobotVelocity();
    var pose = getPose();
    
    // Robot pose and motion
    SmartDashboard.putNumber("Robot/Pose/X", pose.getX());
    SmartDashboard.putNumber("Robot/Pose/Y", pose.getY());
    SmartDashboard.putNumber("Robot/Pose/Rotation", pose.getRotation().getDegrees());
    SmartDashboard.putNumber("Robot/Velocity/vX", chassisSpeeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Robot/Velocity/vY", chassisSpeeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Robot/Velocity/omega", chassisSpeeds.omegaRadiansPerSecond);

    // Gyro data
    SmartDashboard.putNumber("Robot/Gyro/Angle", getGyroAngle());
    SmartDashboard.putNumber("Robot/Gyro/Yaw", getGyroYaw());
    
    // Module states
    for (int i = 0; i < states.length; i++) {
      String prefix = "Swerve/Module" + i;
      // State data
      SmartDashboard.putNumber(prefix + "/Speed", states[i].speedMetersPerSecond);
      SmartDashboard.putNumber(prefix + "/Angle", states[i].angle.getDegrees());
      // Position data
      SmartDashboard.putNumber(prefix + "/Position/Distance", modulePositions[i].distanceMeters);
      SmartDashboard.putNumber(prefix + "/Position/Angle", modulePositions[i].angle.getDegrees());
    }

    // Publish to NetworkTables for simulation consistency
    publisher.set(states);
  }

  @Override
  public void periodic() {
    // Update pose estimation
    poseEstimator.update(
        Rotation2d.fromDegrees(getGyroAngle()),
        swerveDrive.getModulePositions());
    
    // Update all telemetry
    updateTelemetry();
  }

  @Override
  public void simulationPeriodic() {
    double currentTime = Timer.getFPGATimestamp();
    double dt = currentTime - lastSimTime;
    lastSimTime = currentTime;
    
    // Add realistic noise to sensor readings in simulation
    if (Robot.isSimulation()) {
      // Add noise to gyro reading
      double gyroNoise = random.nextGaussian() * Constants.ModuleConstants.Simulation.kGyroNoiseStdDev;
      SmartDashboard.putNumber("Robot/Simulation/GyroNoise", gyroNoise);
      
      // Add noise to encoder readings
      for (int i = 0; i < 4; i++) {
        double encoderNoise = random.nextGaussian() * Constants.ModuleConstants.Simulation.kEncoderNoiseStdDev;
        SmartDashboard.putNumber("Robot/Simulation/Module" + i + "/EncoderNoise", encoderNoise);
      }
      
      // Get current chassis speeds directly
      ChassisSpeeds speeds = swerveDrive.getRobotVelocity();
      
      // Update simulation data
      SmartDashboard.putNumber("Robot/Simulation/SimulatedVx", speeds.vxMetersPerSecond);
      SmartDashboard.putNumber("Robot/Simulation/SimulatedVy", speeds.vyMetersPerSecond);
      SmartDashboard.putNumber("Robot/Simulation/SimulatedOmega", speeds.omegaRadiansPerSecond);
    }
    
    // Update telemetry
    updateTelemetry();
  }

  private void setupSimulation() {
    lastSimTime = Timer.getFPGATimestamp();
  }

}
