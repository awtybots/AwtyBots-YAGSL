// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

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
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
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

  public static final class ElevatorConstants{
    public static final int LeftElevatorCanID = 10;
    public static final int RightElevatorCanID = 11;
  }

  public static final class ArmConstants{
    public static final int ArmLeftCanID = 12;
    public static final int ArmRightCanID = 13;
    public static final int WristCanID = 17;
    public static final int IntakeCanID = 18;
  }

  public static final class FunnelConstants{
    public static final int FunnelWrist = 9;
    public static final int FunnelLIntake = 15;
    public static final int FunnelRIntake = 16;
  }

  public static final class ClimbConstants{
    public static final int ClimbMotor = 21;
  }

  public static final class ElevatorSetpoints{
    public static final int FeederStation = 0;
    public static final int L1 = 15;
    public static final int AlgaeLow = 0;
    public static final int L2 = 40;
    public static final int AlgaeHigh = 50;
    public static final int L3 = 72;
    public static final int L4 = 72;
  }

  public static final class ArmSetpoints{
    public static final double FeederStation = 0;
    public static final double L1 = -3;
    public static final double AlgaeLow = -9;
    public static final double L2 = -3;
    public static final double AlgaeHigh = -9;
    public static final double L3 = -3;
    public static final double L4 = -15;
  }

  public static final class WristSetpoints{
    public static final double FeederStation = -2;
    public static final double L1 = 2;
    public static final double AlgaeLow = -3;
    public static final double L2 = 0;
    public static final double AlgaeHigh = -3;
    public static final double L3 = 0;
    public static final double L4 = -2;
  }

  public static final class FunnelWristSetpoints{
    public static final double FeederStation = -2;
    public static final double Climb = 12;
    public static final double kForward = 0.5;
    public static final double kReverse = -.5;
    public static final double kHold = 0.0;
  }

  public static final class ClimbSetPoints{
    public static final double start = 0;
    public static final double climb = 30;
    public static final double kExtend = 0.5;
    public static final double kRetract = -0.5;
    public static final double kHold = 0.0;
    public static final double kIn = 0.5;
    public static final double kOut = -0.5;
  }

  public static final class FunnelIntakeSetpoints{
    public static final double kForward = 0.5;
    public static final double kReverse = 0.65;
    public static final double kHold = 0.0;
  }

  public static final class IntakeSetpoints{
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
}
