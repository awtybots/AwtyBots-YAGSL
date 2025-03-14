// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

import java.io.IOException;
import java.lang.String;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Optional;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final boolean DebugMode = false;
  public static final double maxSpeed = Units.feetToMeters(18.84);

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8;
    public static final double kMaxAngularSpeed = 2 * Math.PI; // radians per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(28);
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28);
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2;
    public static final double kFrontRightChassisAngularOffset = 0;
    public static final double kBackLeftChassisAngularOffset = Math.PI;
    public static final double kBackRightChassisAngularOffset = Math.PI / 2;

    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1;
    public static final int kRearLeftDrivingCanId = 3;
    public static final int kFrontRightDrivingCanId = 7;
    public static final int kRearRightDrivingCanId = 5;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 4;
    public static final int kFrontRightTurningCanId = 2;
    public static final int kRearRightTurningCanId = 8;

    public static final boolean kGyroReversed = false;
  }

  public static final class ElevatorConstants {
    public static final int LeftElevatorCanID = 10;
    public static final int RightElevatorCanID = 11;
  }

  public static final class ArmConstants {
    public static final int ArmLeftCanID = 12;
    public static final int ArmRightCanID = 13;
    public static final int WristCanID = 17;
    public static final int IntakeCanID = 18;
  }

  public static final class FunnelConstants {
    public static final int FunnelWrist = 9;
    public static final int FunnelLIntake = 15;
    public static final int FunnelRIntake = 16;
  }

  public static final class ClimbConstants {
    public static final int ClimbMotor = 21;
  }

  public static final class ElevatorSetpoints {
    public static final int FeederStation = 0;//10
    public static final int L1 = 10;
    public static final int AlgaeLow = 0;
    public static final int L2 = 50;
    public static final int AlgaeHigh = 57;
    public static final int L3 = 95;
    public static final int L4 = 100;
  }

  public static final class ArmSetpoints {
    public static final double FeederStation = -0.5;
    public static final double L1 = -3;
    public static final double AlgaeLow = -9;
    public static final double L2 = -3;
    public static final double AlgaeHigh = -9;
    public static final double L3 = -6;
    public static final double L4 = -13;
  }

  public static final class WristSetpoints {
    public static final double FeederStation = -4;
    public static final double L1 = -3;
    public static final double AlgaeLow = -15;
    public static final double L2 = -8;
    public static final double AlgaeHigh = -15;
    public static final double L3 = -15;
    public static final double L4 = -8;
  }

  public static final class FunnelWristSetpoints {
    public static final double FeederStation = -2;
    public static final double Climb = 12;
    public static final double kForward = 0.5;
    public static final double kReverse = -.5;
    public static final double kHold = 0.0;
  }

  public static final class ClimbSetPoints {
    public static final double start = 0;
    public static final double climb = 30;
    public static final double kExtend = 0.5;
    public static final double kRetract = -0.5;
    public static final double kHold = 0.0;
    public static final double kIn = 0.5;
    public static final double kOut = -0.5;
  }

  public static final class FunnelIntakeSetpoints {
    public static final double kForward = 0.20;
    public static final double kReverse = 0.30;
    public static final double kHold = 0.0;
  }

  public static final class IntakeSetpoints {
    public static final double kForward = -.6;
    public static final double kReverse = .6;
    public static final double kHold = .25;
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14;

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60;
    public static final double kWheelDiameterMeters = 0.0762;
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15);
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction;
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double DEADBAND = 0.2;
    public static final double kTriggerThreshold = 0.2;
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784;
  }

  public static final class VisionConstants {
    public static final class Coral {
      public static final int[] redReefIds = { 6, 7, 8, 9, 10, 11 };
      public static final int[] blueReefIds = { 17, 18, 19, 20, 21, 22 };

      public static final int[] allReefIds = { 6, 7, 8, 9, 10, 11, 17, 18, 19, 20, 21, 22 };

      public static final int[] nonReefIds = { 1, 2, 3, 4, 5, 12, 13, 14, 15, 16 };

      public static final Map<Integer, Pose2d[]> redReefScoringPoses = new HashMap<>();
      public static final Map<Integer, Pose2d[]> blueReefScoringPoses = new HashMap<>();

      static {
        // Populate red alliance scoring positions
        redReefScoringPoses.put(6, new Pose2d[] {
            new Pose2d(13.690, 2.664, Rotation2d.fromDegrees(37.619)), // Tag 6 Left Bar
            new Pose2d(13.834, 2.533, Rotation2d.fromDegrees(-59.534)) // Tag 6 Right Bar
        });

        redReefScoringPoses.put(7, new Pose2d[] {
            new Pose2d(8.5, 2.5, Rotation2d.fromDegrees(180)), // Tag 7 Left Bar
            new Pose2d(8.5, 3.0, Rotation2d.fromDegrees(180)) // Tag 7 Right Bar
        });

        // Populate blue alliance scoring positions
        blueReefScoringPoses.put(17, new Pose2d[] {
            new Pose2d(2.0, 2.5, Rotation2d.fromDegrees(0)), // Tag 17 Left Bar
            new Pose2d(2.0, 3.0, Rotation2d.fromDegrees(0)) // Tag 17 Right Bar
        });

        blueReefScoringPoses.put(18, new Pose2d[] {
            new Pose2d(4.0, 2.5, Rotation2d.fromDegrees(0)), // Tag 18 Left Bar
            new Pose2d(4.0, 3.0, Rotation2d.fromDegrees(0)) // Tag 18 Right Bar
        });
      }

      public static AprilTagFieldLayout aprilTagFieldLayout;

      static {
        try {
          aprilTagFieldLayout = AprilTagFieldLayout
              .loadFromResource(AprilTagFields.k2025ReefscapeAndyMark.m_resourceFile);
          aprilTagFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);
        } catch (IOException e) {
          e.printStackTrace();
        }
      }

      /**
       * Get the best scoring pose based on the detected AprilTag and bar alignment.
       * 
       * @param aprilTagID The detected AprilTag ID
       * @param alignLeft  True for the left bar, False for the right bar
       * @return The target Pose2d for alignment, or a default Pose2d if the tag isn't
       *         found
       */
      public static Pose2d getBestReefPose(int tagID, boolean alignLeft) {
        Optional<Pose3d> tagPoseOpt = aprilTagFieldLayout.getTagPose(tagID);

        if (tagPoseOpt.isPresent()) {
          Pose2d tagPose = tagPoseOpt.get().toPose2d();

          // Apply an offset based on which side the robot needs to align with
          double xOffset = 0.45; // Move forward slightly to align better
          double yOffset = alignLeft ? 0.3 : -0.3; // Adjust laterally based on alignment

          return tagPose.plus(new Transform2d(xOffset, yOffset, new Rotation2d(Math.PI)));
        }

        // Return a default pose if the tag is not found
        return new Pose2d();
      }

      public static final List<String> cameraNames = List.of(
          "OV9281" // Front Camera (AprilTag Limelight)
      // "Arducam_Left", // Left Camera
      // "Arducam_Right" // Right Camera
      );

      public static final List<Transform3d> cameraPoses = List.of(
          new Transform3d( // Front Camera (Limelight)
              new Translation3d(0.25, -0.25, 0.381),
              new Rotation3d(0, Units.degreesToRadians(0), 0))
      // new Transform3d( // Left Camera
      // new Translation3d(0.3, 0.2, 0.35),
      // new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(90))),
      // new Transform3d( // Right Camera
      // new Translation3d(0.3, -0.2, 0.35),
      // new Rotation3d(0, Units.degreesToRadians(0), Units.degreesToRadians(-90)))
      );

      // Camera's
      // public static final String limelightAprilTagCamera = "OV9281";
    
      public static final double maxForwardSpeed = 0.5; // Max forward/backward speed (m/s)
      public static final double maxStrafeSpeed = 0.5; // Max strafe speed (m/s)
      public static final double maxRotationSpeed = 0.5; // Max rotation speed (rad/s)
      // PID Constants for Forward/Backward Translation
      public static final double TRANSLATION_kP = 0.8;
      public static final double TRANSLATION_kI = 0.08;
      public static final double TRANSLATION_kD = 0.05;

      // PID Constants for Strafing (Side-to-Side Movement)
      public static final double STRAFE_kP = 1.2;
      public static final double STRAFE_kI = 0.0;
      public static final double STRAFE_kD = 0.03;

      // PID Constants for Rotation (Turning to Face Target)
      public static final double ROTATION_kP = 0.2;
      public static final double ROTATION_kI = 0.0;
      public static final double ROTATION_kD = 0.02;

      // Distance Thresholds (How Close Should the Robot Get?)
      public static final double DISTANCE_THRESHOLD = 0.5; // Target distance from AprilTag
      public static final double TRANSLATION_TOLERANCE = 0.5; // Allowable translation error in meters
      public static final double STRAFE_TOLERANCE = 0.05; // 5 cm strafe tolerance
      public static final double ROTATION_THRESHOLD = 0.0; // 5 cm strafe tolerance
      public static final double ROTATION_TOLERANCE = 5; // 2 degrees of rotation tolerance

      // Profiled PID Constraints (Velocity & Acceleration Limits)
      public static final edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints TRANSLATION_CONSTRAINTS = new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
          1.5, 1.0); // Max: 1.5 m/s, Accel: 1.0 m/s²

      public static final edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints STRAFE_CONSTRAINTS = new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
          1.5, 1.0); // Same as translation

      public static final edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints ROTATION_CONSTRAINTS = new edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints(
          Units.degreesToRadians(180), Units.degreesToRadians(90)); // Max: 180°/s, Accel: 90°/s²

      // Alignment Offsets (Lateral Adjustments for Reef Bars)
      public static final double LEFT_OFFSET = 0.43; // Adjust this to align with left reef bar
      public static final double RIGHT_OFFSET = 0.0; // Adjust this to align with right reef bar
    }

    public static final class FeederStation {
      // Camera's
      public static final String FeederSationLimelightAprilTagCamera = "Arducam_OV9782_USB_Camera";
      public static final double cameraMountX = 1; // How far forwards/backwards is the camera mounted from center
      public static final double cameraMountY = 0.3; // How far left/right is the camera mounted from center
      public static final double cameraMountHeight = 0.02; // This is in meters
      public static final double cameraMountAngle = 0.0; // This is in degrees
      public static final double maxForwardSpeed = 2; // Max forward/backward speed (m/s)
      public static final double maxStrafeSpeed = 3; // Max strafe speed (m/s)
      public static final double maxRotationSpeed = 1; // Max rotation speed (rad/s)
      public static final double DistancekP = 1;
      public static final double DistancekI = 0.0;
      public static final double DistancekD = 0.05;
      public static final double StrafekP = 0.9;
      public static final double StrafekI = 0.0;
      public static final double StrafekD = 0.03;
      public static final double RotationkP = 0.8;
      public static final double RotationkI = 0.0;
      public static final double RotationkD = 0.02;
      public static final double targetDistanceMeters = 2.0; // Default: 2 meter away from target
      public static final double targetDistanceMetersLevel4 = 1.0; // Target distance level 4
      public static final double distanceTolerance = 0.02; // How off are we willing to tolerate
      public static final double strafeTolerance = 0.05; // 5 cm tolerance
      public static final double rotationTolerance = 2.0; // 2 degrees for turning left and right
      public static final double leftOffsetMeters = 1.0; // Adjust how far left to align
      public static final double rightOffsetMeters = 1.0; // Adjust how far right to align

    }

    public static final class Algae {
      // Camera's
      public static final String orangePIAlgaeCamera = "Arducam_OV9782_USB_Camera";
      public static final double cameraMountX = 0.5; // How far forwards/backwards is the camera mounted from center
      public static final double cameraMountY = 0.3; // How far left/right is the camera mounted from center
      public static final double cameraMountHeight = 0.02; // This is in meters
      public static final double cameraMountAngle = 0.0; // This is in degrees
      public static final double maxForwardSpeed = 2; // Max forward/backward speed (m/s)
      public static final double maxStrafeSpeed = 3; // Max strafe speed (m/s)
      public static final double maxRotationSpeed = 1; // Max rotation speed (rad/s)
      public static final double DistancekP = 1;
      public static final double DistancekI = 0.0;
      public static final double DistancekD = 0.05;
      public static final double StrafekP = 0.9;
      public static final double StrafekI = 0.0;
      public static final double StrafekD = 0.03;
      public static final double RotationkP = 0.8;
      public static final double RotationkI = 0.0;
      public static final double RotationkD = 0.02;
      public static final double targetDistanceMeters = 0.3; // Default: 2 meter away from target
      public static final double targetDistanceMetersLevel4 = 1.0; // Target distance level 4
      public static final double distanceTolerance = 1; // How off are we willing to tolerate
      public static final double strafeTolerance = 0.05; // 5 cm tolerance
      public static final double rotationTolerance = 2.0; // 2 degrees for turning left and right
      public static final double leftOffsetMeters = 1.0; // Adjust how far left to align
      public static final double rightOffsetMeters = 1.0; // Adjust how far right to align

    }

  }

}
