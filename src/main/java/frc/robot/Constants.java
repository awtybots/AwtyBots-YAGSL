// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.NeoMotorConstants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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
  
    public static final int FunnelLIntake = 22;

  }

  public static final class ClimbConstants{
    public static final int ClimbMotor = 21;
  }

  public static final class ElevatorSetpoints{
    public static final int FeederStation = 0;
    public static final int L1 = 15;
    public static final int AlgaeLow = 0;
    public static final int L2 = 47;
    public static final int AlgaeHigh = 42;
    public static final int L3 = 0 ;
    public static final int L4 = 70;
    public static final int Barge = 27;
  }

  public static final class ArmSetpoints{
    public static final double FeederStation = -1.9;//-1.9 //
    public static final double L1 = -3; //-3
    public static final double AlgaeLow = -9;
    public static final double L2 = -3;
    public static final double AlgaeHigh = -9;
    public static final double L3 = -13.583; //-12.5
    public static final double L4 = -13;
    public static final double Barge = -14;
    // public static final double FeederStation = 1.375;//-1.9 //
    // public static final double L1 = 0.224; //-3
    // public static final double AlgaeLow = 0.1;
    // public static final double L2 = 0.224;
    // public static final double AlgaeHigh = 0.1;
    // public static final double L3 = 0.954;
    // public static final double L4 = 0.9775;
    // public static final double Barge = 0.1;

  }
  public static final class WristSetpoints{
    public static final double FeederStation = 0.55;
    public static final double L1 = 0.8;
    public static final double AlgaeLow = .27;
    public static final double L2 = .5;
    public static final double AlgaeHigh = .27;
    public static final double L3 = .66;
    public static final double L4 = .61;
    public static final double Barge = .9;
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
    public static final double kIn = 1;
    public static final double kOut = -1;
  }

  public static final class FunnelIntakeSetpoints{
    public static final double kForward = 0.20;
    public static final double kReverse = 0.30;
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
  // AutoAlign constants
  // Raise these gains if the chassis reacts slowly, lower if it oscillates or overshoots.
  public static final double X_REEF_ALIGNMENT_P = 1.5; //1
  public static final double Y_REEF_ALIGNMENT_P = 2.3;//1.74
  public static final double ROT_REEF_ALIGNMENT_P = 0.1;

  // Shift these setpoints when the robot stops short, crashes the reef, or parks off-center.
  public static final double ROT_SETPOINT_REEF_ALIGNMENT = -0.1; // Rotation
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 0.05;
  public static final double X_SETPOINT_REEF_ALIGNMENT = 0.06;//-0.43 // Vertical pose
  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.08;
  public static final double Y_L_SETPOINT_REEF_ALIGNMENT = 0.05; // -0.359 Horizontal pose
  public static final double Y_R_SETPOINT_REEF_ALIGNMENT = 0.3; // Horizontal pose
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.1;


  // Extend this wait if brief vision dropouts abort alignment, shorten to bail sooner.
  public static final double DONT_SEE_TAG_WAIT_TIME = 1;
  public static final double POSE_VALIDATION_TIME = 0.4;

  // public static final class LimelightConstants {

  //   public static void initializeLimelight() {
  //     // Set a custom crop window for improved performance (-1 to 1 for each value)
  //     LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

  //     // Change the camera pose relative to robot center (x forward, y left, z up,
  //     // degrees)
  //     LimelightHelpers.setCameraPose_RobotSpace("",
  //         0.5, // Forward offset (meters)
  //         0.0, // Side offset (meters)
  //         0.5, // Height offset (meters)
  //         0.0, // Roll (degrees)
  //         30.0, // Pitch (degrees)
  //         0.0 // Yaw (degrees)
  //     );

  //     // Set AprilTag offset tracking point (meters)
  //     LimelightHelpers.setFiducial3DOffset("",
  //         0.0, // Forward offset
  //         0.0, // Side offset
  //         0.5 // Height offset
  //     );

  //     // Configure AprilTag detection
  //     LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] { 1, 2, 3, 4, 5, 6, 7, 8 }); // Only track these tag
  //                                                                                              // IDs
  //     LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half resolution for improved framerate
  //                                                                // and reduced range
  //   }
  // }
}
