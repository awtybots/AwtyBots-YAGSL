// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

import java.lang.String;

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

  public static final class DrivebaseConstants {
    public static double DriveFastScale = 1;
    public static double DrivePrecisionScale = 0.35;
    // Hold time on motor brakes when disabled
    public static final double WHEEL_LOCK_TIME = 10.0; // seconds

    public enum TargetSide {
      LEFT, RIGHT
    };

    // robot camera offsets need to be correct with bumper so the
    // align to reef works correctly, the reef poles are 6.5 inches from the
    // center of the april tag
    public static double ReefLeftYOffset = Units.inchesToMeters(-9.5);
    public static double ReefRightYOffset = Units.inchesToMeters(4);
    public static double ReefXDistance = Units.inchesToMeters(25);
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

    public static final int FunnelLIntake = 22;

  }

  public static final class ClimbConstants {
    public static final int ClimbMotor = 21;
  }

  public static final class ElevatorSetpoints {
    public static final int FeederStation = 4;
    public static final int L1 = 10;
    public static final int AlgaeLow = 0;
    public static final int L2 = 50;
    public static final int AlgaeHigh = 57;
    public static final int L3 = 95;
    public static final int L4 = 100;
  }

  public static final class ArmSetpoints {
    public static final double FeederStation = -1.5;
    public static final double L1 = -3;
    public static final double AlgaeLow = -9;
    public static final double L2 = -3;
    public static final double AlgaeHigh = -9;
    public static final double L3 = -6;
    public static final double L4 = -13;

  }

  public static final class WristSetpoints {
    public static final double FeederStation = -8.2;
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
    public static final double kIn = 1;
    public static final double kOut = -1;
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
    public static final boolean DRIVEWITHVISION = true;

    public static final class Coral {

      public static final double maxForwardSpeed = 0.5; // Max forward/backward speed (m/s)
      public static final double maxStrafeSpeed = 0.5; // Max strafe speed (m/s)
      public static final double maxRotationSpeed = 0.5; // Max rotation speed (rad/s)
      // PID Constants for Forward/Backward Translation
      public static final double TRANSLATION_kP = 2.3;
      public static final double TRANSLATION_kI = 0.0;
      public static final double TRANSLATION_kD = 0.001;

      // PID Constants for Strafing (Side-to-Side Movement)
      public static final double STRAFE_kP = 0.1;
      public static final double STRAFE_kI = 0.0;
      public static final double STRAFE_kD = 0.001;

      // PID Constants for Rotation (Turning to Face Target)
      public static final double ROTATION_kP = 0.08;
      public static final double ROTATION_kI = 0.0;
      public static final double ROTATION_kD = 0.001;

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
