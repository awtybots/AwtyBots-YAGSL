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

  public static final double maxSpeed = Units.feetToMeters(18.84); // Maximum achievable drivetrain speed in meters per
                                                                   // second

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0; // USB port index for driver controller
  }

  public static final class DriveConstants {
    // Driving Parameters - Note that these are not the maximum capable speeds of
    // the robot, rather the allowed maximum speeds
    public static final double kMaxSpeedMetersPerSecond = 4.8; // Maximum commanded swerve translation speed
    public static final double kMaxAngularSpeed = 2 * Math.PI; // Maximum commanded chassis rotation speed in radians
                                                               // per second

    // Chassis configuration
    public static final double kTrackWidth = Units.inchesToMeters(28); // Lateral distance between left and right module
                                                                       // centers
    // Distance between centers of right and left wheels on robot
    public static final double kWheelBase = Units.inchesToMeters(28); // Longitudinal distance between front and rear
                                                                      // module centers
    // Distance between front and back wheels on robot
    public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics( // Kinematic layout of
                                                                                            // swerve modules
        new Translation2d(kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
        new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Angular offsets of the modules relative to the chassis in radians
    public static final double kFrontLeftChassisAngularOffset = -Math.PI / 2; // Offset aligning front-left module
                                                                              // encoder to chassis frame
    public static final double kFrontRightChassisAngularOffset = 0; // Offset aligning front-right module encoder to
                                                                    // chassis frame
    public static final double kBackLeftChassisAngularOffset = Math.PI; // Offset aligning back-left module encoder to
                                                                        // chassis frame
    public static final double kBackRightChassisAngularOffset = Math.PI / 2; // Offset aligning back-right module
                                                                             // encoder to chassis frame

    // SPARK MAX CAN IDs
    // SPARK MAX CAN IDs
    public static final int kFrontLeftDrivingCanId = 1; // CAN ID for front-left drive motor
    public static final int kRearLeftDrivingCanId = 3; // CAN ID for rear-left drive motor
    public static final int kFrontRightDrivingCanId = 7; // CAN ID for front-right drive motor
    public static final int kRearRightDrivingCanId = 5; // CAN ID for rear-right drive motor

    public static final int kFrontLeftTurningCanId = 6; // CAN ID for front-left steering motor
    public static final int kRearLeftTurningCanId = 4; // CAN ID for rear-left steering motor
    public static final int kFrontRightTurningCanId = 2; // CAN ID for front-right steering motor
    public static final int kRearRightTurningCanId = 8; // CAN ID for rear-right steering motor

    public static final boolean kGyroReversed = false; // Whether gyro readings need to be inverted
  }

  public static final class ElevatorConstants {
    public static final int LeftElevatorCanID = 10; // CAN ID for left elevator motor controller
    public static final int RightElevatorCanID = 11; // CAN ID for right elevator motor controller
  }

  public static final class ArmConstants {
    public static final int ArmLeftCanID = 12; // CAN ID for left arm motor controller
    public static final int ArmRightCanID = 13; // CAN ID for right arm motor controller
    public static final int WristCanID = 17; // CAN ID for wrist motor controller
    public static final int IntakeCanID = 18; // CAN ID for primary intake motor controller
  }

  public static final class FunnelConstants {

    public static final int FunnelLIntake = 22; // CAN ID for funnel intake motor

  }

  public static final class ClimbConstants {
    public static final int ClimbMotor = 21; // CAN ID for climb motor controller
  }

  public static final class ElevatorSetpoints {
    public static final int FeederStation = 0; // Elevator position for human player station pickup
    public static final int L1 = 15; // Elevator position for level 1 coral scoring
    public static final int AlgaeLow = 0; // Elevator position for low algae removal
    public static final int L2 = 47; // Elevator position for level 2 coral scoring
    public static final int AlgaeHigh = 42; // Elevator position for high algae removal
    public static final int L3 = 0; // Elevator position placeholder for level 3 coral scoring
    public static final int L4 = 73; // Elevator position for level 4 coral scoring
    public static final int Barge = 73; // Elevator position for barge scoring
  }

  public static final class ArmSetpoints {
    public static final double FeederStation = -1.3; // Arm angle for human player station pickup (was -1.9)
    public static final double L1 = -3; // Arm angle for level 1 coral scoring
    public static final double AlgaeLow = -9; // Arm angle for removing low algae
    public static final double L2 = -3; // Arm angle for level 2 coral scoring
    public static final double AlgaeHigh = -9; // Arm angle for removing high algae
    public static final double L3 = -13.583; // Arm angle for level 3 coral scoring (previously -12.5)
    public static final double L4 = -13; // Arm angle for level 4 coral scoring
    public static final double Barge = -14; // Arm angle for barge scoring
    // public static final double FeederStation = 1.375;//-1.9 //
    // public static final double L1 = 0.224; //-3
    // public static final double AlgaeLow = 0.1;
    // public static final double L2 = 0.224;
    // public static final double AlgaeHigh = 0.1;
    // public static final double L3 = 0.954;
    // public static final double L4 = 0.9775;
    // public static final double Barge = 0.1;

  }

  public static final class WristSetpoints {
    public static final double FeederStation = 0.55; // Wrist angle for human player station pickup
    public static final double L1 = 0.8; // Wrist angle for level 1 coral scoring
    public static final double AlgaeLow = .27; // Wrist angle for removing low algae
    public static final double L2 = .5; // Wrist angle for level 2 coral scoring
    public static final double AlgaeHigh = .27; // Wrist angle for removing high algae
    public static final double L3 = .66; // Wrist angle for level 3 coral scoring
    public static final double L4 = .67; // Wrist angle for level 4 coral scoring
    public static final double Barge = .8; // Wrist angle for barge scoring
  }

  public static final class FunnelWristSetpoints {
    public static final double FeederStation = -2; // Funnel wrist angle when staged at human player station
    public static final double Climb = 12; // Funnel wrist angle to clear climb configuration
    public static final double kForward = 0.5; // Funnel wrist power to swing forward
    public static final double kReverse = -.5; // Funnel wrist power to swing backward
    public static final double kHold = 0.0; // Funnel wrist power to hold position
  }

  public static final class ClimbSetPoints {
    public static final double start = 0; // Initial climb mechanism setpoint
    public static final double climb = 30; // Extended climb mechanism setpoint
    public static final double kExtend = 0.5; // Motor power to extend climb arms
    public static final double kRetract = -0.5; // Motor power to retract climb arms
    public static final double kHold = 0.0; // Motor power to hold climb position
    public static final double kIn = 1; // Pneumatic state for pulling climb arms in
    public static final double kOut = -1; // Pneumatic state for pushing climb arms out
  }

  public static final class FunnelIntakeSetpoints {
    public static final double kForward = 0.20; // Motor output to run funnel intake forward
    public static final double kReverse = 0.30; // Motor output to run funnel intake in reverse
    public static final double kHold = 0.0; // Motor output to hold funnel intake stationary
  }

  public static final class IntakeSetpoints {
    public static final double kForward = -.6; // Intake power to collect game pieces
    public static final double kReverse = .6; // Intake power to eject game pieces
    public static final double kHold = .25; // Intake power to retain a game piece
    public static final double kFastForward = -1; // Maximum intake power for rapid collection
    public static final double kFastBackward = 1; // Maximum intake power for rapid ejection
  }

  public static final class CoralToleranceConstants {
    public static final double ELEVATOR_TOLERANCE = 0.5; // Allowable elevator position error before declaring on target
    public static final double ARM_TOLERANCE = 0.5; // Allowable arm position error before declaring on target
    public static final double WRIST_TOLERANCE = 0.05; // Allowable wrist position error before declaring on target
  }

  public static final class SafetyConstants {
    public static final double BACKUP_RELEASE_DISTANCE_METERS = Units.inchesToMeters(16); // Distance to reverse before
                                                                                          // releasing game piece
    public static final double LOCK_REMINDER_INTERVAL_SECONDS = 0.75; // Interval between lock reminder prompts
  }

  public static final class ModuleConstants {
    // The MAXSwerve module can be configured with one of three pinion gears: 12T,
    // 13T, or 14T. This changes the drive speed of the module (a pinion gear with
    // more teeth will result in a robot that drives faster).
    public static final int kDrivingMotorPinionTeeth = 14; // Selected drive motor pinion tooth count

    // Calculations required for driving motor conversion factors and feed forward
    public static final double kDrivingMotorFreeSpeedRps = NeoMotorConstants.kFreeSpeedRpm / 60; // Motor free speed in
                                                                                                 // rotations per second
    public static final double kWheelDiameterMeters = 0.0762; // Swerve wheel diameter in meters
    public static final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI; // Swerve wheel circumference
                                                                                           // in meters
    // 45 teeth on the wheel's bevel gear, 22 teeth on the first-stage spur gear, 15
    // teeth on the bevel pinion
    public static final double kDrivingMotorReduction = (45.0 * 22) / (kDrivingMotorPinionTeeth * 15); // Overall drive
                                                                                                       // gear reduction
                                                                                                       // ratio
    public static final double kDriveWheelFreeSpeedRps = (kDrivingMotorFreeSpeedRps * kWheelCircumferenceMeters)
        / kDrivingMotorReduction; // Predicted free speed at wheel surface
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0; // USB port index for driver controller
    public static final int kOperatorControllerPort = 1; // USB port index for operator controller
    public static final double DEADBAND = 0.2; // Deadband applied to joystick axes
    public static final double kTriggerThreshold = 0.2; // Threshold for treating triggers as active
  }

  public static final class NeoMotorConstants {
    public static final double kFreeSpeedRpm = 6784; // Free speed of a NEO motor in RPM
  }

  // AutoAlign constants
  // Raise these gains if the chassis reacts slowly, lower if it oscillates or
  // overshoots.
  public static final double X_REEF_ALIGNMENT_P = 1.9; // Proportional gain for X-axis reef alignment
  public static final double Y_REEF_ALIGNMENT_P = 2.3; // Proportional gain for Y-axis reef alignment (previously 1.74)
  public static final double ROT_REEF_ALIGNMENT_P = 0.07; // Proportional gain for rotational reef alignment
  public static final boolean USE_AUTO_ALIGNMENT_FAST_APPROACH = false; // Turn fast appraoch for auto align
  public static final double AUTO_ALIGNMENT_FAST_APPROACH_DISTANCE_METERS = 0.60; // Distance where we switch from max
                                                                                  // speed to PID control
  public static final double AUTO_ALIGNMENT_FAST_APPROACH_SPEED = 1.2; // Fast approach speed in m/s when far from the
                                                                       // reef
  public static final double AUTO_ALIGNMENT_LINEAR_SETTLE_SPEED = 0.12; // Robot linear speed considered "settled"
  public static final double AUTO_ALIGNMENT_ANGULAR_SETTLE_SPEED = 0.6; // Robot angular speed considered "settled"
  public static final double AUTO_ALIGNMENT_X_CLOSE_ENOUGH = 0.12; // Allowable X error to finish auto align early
  public static final double AUTO_ALIGNMENT_Y_CLOSE_ENOUGH = 0.18; // Allowable Y error to finish auto align early
  public static final double AUTO_ALIGNMENT_ROT_CLOSE_ENOUGH = 1; // Allowable rotation error (degrees) for completion

  // Shift these setpoints when the robot stops short, crashes the reef, or parks
  // off-center.
  public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0; // Desired robot heading when aligned to the reef
  public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1; // Allowable heading error while aligning
  public static final double X_SETPOINT_REEF_ALIGNMENT = 0.06; // Desired X offset from reef for scoring (previously
                                                               // -0.43)
  public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.08; // Acceptable X error when aligning
  public static final double Y_L_SETPOINT_REEF_ALIGNMENT = 0.05; // Desired Y offset when approaching left reef side
                                                                 // (was -0.359)
  public static final double Y_R_SETPOINT_REEF_ALIGNMENT = 0.275; // Desired Y offset when approaching right reef side
  public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.1; // Acceptable Y error during alignment

  // Extend this wait if brief vision dropouts abort alignment, shorten to bail
  // sooner.
  public static final double DONT_SEE_TAG_WAIT_TIME = 0.4; // Time to continue aligning after vision tag loss
  public static final double POSE_VALIDATION_TIME = 0.07; // Duration a pose measurement must remain valid
  public static final double POSE_LOSS_GRACE_PERIOD = 0.2; // Allowed vision dropout time before aborting alignment

  // Dashboard throttling
  public static final boolean LIMIT_DASHBOARD_PERIODIC_UPDATES = false; // Enable throttling of dashboard updates
  public static final int DASHBOARD_UPDATE_PERIOD_CYCLES = 10; // Number of periodic loops between dashboard refreshes

  // public static final class LimelightConstants {

  // public static void initializeLimelight() {
  // // Set a custom crop window for improved performance (-1 to 1 for each value)
  // LimelightHelpers.setCropWindow("", -0.5, 0.5, -0.5, 0.5);

  // // Change the camera pose relative to robot center (x forward, y left, z up,
  // // degrees)
  // LimelightHelpers.setCameraPose_RobotSpace("",
  // 0.5, // Forward offset (meters)
  // 0.0, // Side offset (meters)
  // 0.5, // Height offset (meters)
  // 0.0, // Roll (degrees)
  // 30.0, // Pitch (degrees)
  // 0.0 // Yaw (degrees)
  // );

  // // Set AprilTag offset tracking point (meters)
  // LimelightHelpers.setFiducial3DOffset("",
  // 0.0, // Forward offset
  // 0.0, // Side offset
  // 0.5 // Height offset
  // );

  // // Configure AprilTag detection
  // LimelightHelpers.SetFiducialIDFiltersOverride("", new int[] { 1, 2, 3, 4, 5,
  // 6, 7, 8 }); // Only track these tag
  // // IDs
  // LimelightHelpers.SetFiducialDownscalingOverride("", 2.0f); // Process at half
  // resolution for improved framerate
  // // and reduced range
  // }
  // }
}
