// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

  public static final double maxSpeed = Units.feetToMeters(15.76);

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

  public static final class ElevatorSetpoints {
    public static final int FeederStation = 0;
    public static final int L1 = 15;
    public static final int AlgaeLow = 0;
    public static final int L2 = 40;
    public static final int AlgaeHigh = 50;
    public static final int L3 = 72;
    public static final int L4 = 72;
  }

  public static final class ArmSetpoints {
    public static final double FeederStation = 0;
    public static final double L1 = -3;
    public static final double AlgaeLow = -9;
    public static final double L2 = -3;
    public static final double AlgaeHigh = -9;
    public static final double L3 = -3;
    public static final double L4 = -15;
  }

  public static final class WristSetpoints {
    public static final double FeederStation = -1;
    public static final double L1 = 3;
    public static final double AlgaeLow = -3;
    public static final double L2 = 0;
    public static final double AlgaeHigh = -3;
    public static final double L3 = 0;
    public static final double L4 = -2;
  }

  public static final class FunnelWristSetpoints {
    public static final double FeederStation = -2;
    public static final double Climb = 12;
  }

  public static final class FunnelIntakeSetpoints {
    public static final double kForward = 0.5;
    public static final double kReverse = 0.65;
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
    public static final double kFreeSpeedRpm = 5676;
  }

  public static final class VisionConstants {
    public static final class Coral {
      // Camera's
      public static final String limelightAprilTagCamera = "Arducam_OV9782_USB_Camera";
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
      public static final double targetDistanceMeters = 2.0; // Default: 2 meter away from target
      public static final double targetDistanceMetersLevel4 = 1.0; // Target distance level 4
      public static final double distance_tolerance = 0.02; // How off are we willing to tolerate
      public static final double strafe_tolerance = 0.05; // 5 cm tolerance
      public static final double rotation_tolerance = 2.0; // 2 degrees for turning left and right
      public static final double leftOffsetMeters = 1.0; // Adjust how far left to align
      public static final double rightOffsetMeters = 1.0; // Adjust how far right to align
    }

    public static final class FeederStation {
      // Camera's
      public static final String FeederSationLimelightAprilTagCamera = "Arducam_OV9782_USB_Camera";
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
      public static final double targetDistanceMeters = 2.0; // Default: 2 meter away from target
      public static final double targetDistanceMetersLevel4 = 1.0; // Target distance level 4
      public static final double distance_tolerance = 0.02; // How off are we willing to tolerate
      public static final double strafe_tolerance = 0.05; // 5 cm tolerance
      public static final double rotation_tolerance = 2.0; // 2 degrees for turning left and right
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
      public static final double targetDistanceMeters = 2.0; // Default: 2 meter away from target
      public static final double targetDistanceMetersLevel4 = 1.0; // Target distance level 4
      public static final double distance_tolerance = 0.02; // How off are we willing to tolerate
      public static final double strafe_tolerance = 0.05; // 5 cm tolerance
      public static final double rotation_tolerance = 2.0; // 2 degrees for turning left and right
      public static final double leftOffsetMeters = 1.0; // Adjust how far left to align
      public static final double rightOffsetMeters = 1.0; // Adjust how far right to align

    }

  }

}
