# AwtyBots-YAGSL

Modern FRC 2025 robot project using WPILib’s Command-Based framework with a YAGSL swerve drive, PathPlanner autonomous paths, Limelight AprilTag alignment, and REV Spark Flex controllers for mechanisms.

This README orients new contributors to the project layout, subsystems, controls, configs, and tuning workflow.

**Quick Links**
- Build/deploy: Use the WPILib VS Code palette (see below)
- Autos and paths: `src/main/deploy/pathplanner`
- Swerve config (YAGSL): `src/main/deploy/swerve`
- Main code: `src/main/java/frc/robot`

## Project Structure
- `src/main/java/frc/robot`
  - `Robot.java`: WPILib lifecycle. Starts camera, runs scheduler, sets auto.
  - `RobotContainer.java`: Wires subsystems, default/trigger bindings, NamedCommands, Auto chooser.
  - `Constants.java`: Central constants (operator ports, setpoints, alignment gains, etc.).
  - `Configs.java`: Programmatic Spark Flex/Max configs (idle mode, current limits, PID/MAXMotion, etc.).
  - `LimelightHelpers.java`: Limelight utility (2025 API), used for tag-based alignment.
  - `estimation/swervedriveodometry2.java`: Wrapper around `SwerveDrivePoseEstimator` with added telemetry.
  - `subsystems/`: Robot subsystems
    - `SwerveSubsystem.java`: YAGSL swerve drive + PathPlanner integration + navX gyro.
    - `CoralSubsystem.java`: Elevator (Spark Flex, MAXMotion) with discrete setpoints; coordination flags.
    - `EndE.java`: Coral intake using LaserCan to auto-detect game piece.
    - `FunnelIntake.java`: Funnel intake roller.
    - `Climber.java`: Simple climb motor control and manual commands.
    - `AlgaeArmSubsystem.java`: Arm control (present; some usage and wrist are commented out).
  - `commands/`: Custom commands
    - `LAlignToReefTagRelative.java`, `RAlignToReefTagRelative.java`: Limelight-based tag-relative reef alignment.
    - `Autos.java`: Utility placeholder (auto chooser uses PathPlanner’s builder).
- `src/main/deploy`
  - `swerve/`: YAGSL configuration
    - `swervedrive.json`, `controllerproperties.json`
    - `modules/` with `physicalproperties.json`, `pidfproperties.json`, and per-module JSONs.
  - `pathplanner/`: GUI-generated autos, paths, and navgrid used by PathPlanner.
- `vendordeps/`: Vendor libraries (YAGSL, REVLib, Phoenix 5/6, PathPlannerLib, Studica/navX, Grapple/LaserCan, etc.).

## Key Libraries
- **YAGSL**: Swerve framework + JSON configs for modules/drive; provides `SwerveDrive`, kinematics, and helpers.
- **PathPlannerLib**: Autonomous path following + event markers + pathfinding; auto chooser configured via GUI settings.
- **LimelightHelpers (2025)**: AprilTag pose, MegaTag2, and utility network table access; used for reef alignment.
- **REVLib (Spark Flex)**: Motor control, PID + MAXMotion, current limits, voltage compensation via `Configs`.
- **Studica/navX**: Gyro used for field-oriented swerve and odometry.
- **CTRE Phoenix**: Included in vendordeps; swerve uses YAGSL and REV, but CTRE is available if needed.

## Swerve Drive (YAGSL)
- Config files: `src/main/deploy/swerve` control geometry, encoder offsets, PIDF, and conversion factors.
- Initialization: `SwerveSubsystem` builds a `SwerveDrive` via `new SwerveParser(deploy/swerve).createSwerveDrive(...)`.
- Gyro: navX (SPI MXP). `getGyroYaw()` feeds field-oriented control and odometry.
- Odometry: dual – standard `SwerveDrivePoseEstimator` and custom `swervedriveodometry2` for added logging.
- PathPlanner integration
  - Config loaded from GUI (`RobotConfig.fromGUISettings()`), `AutoBuilder.configure(...)` wired to `SwerveDrive`.
  - Red/Blue mirroring based on `DriverStation.getAlliance()`.
  - Pre-warm pathfinding: `PathfindingCommand.warmupCommand().schedule()`.

Tip: The numeric CAN IDs in `Constants.DriveConstants` are from a MAXSwerve template and not used by YAGSL. YAGSL reads IDs/offsets from the deploy JSONs.

## Subsystems & Setpoints
### SwerveSubsystem
- Field-oriented and robot-oriented driving methods, drive-to-pose helper, and `zeroHeadingCommand()`.
- Heading bias + initial heading configured to support field-oriented defaults.

### CoralSubsystem (Elevator and setpoint orchestration)
- Hardware: Two Spark Flex motors (`LeftElevatorCanID = 10`, `RightElevatorCanID = 11`) with right following left.
- Control: `MAXMotion` position control; PID values set in `Configs.CoralSubsystem`.
- Setpoints (`Constants.ElevatorSetpoints`): `FeederStation`, `L1`, `L2`, `L3`, `L4`, `Barge`, plus algae pickup heights.
- Coordination flags:
  - `ElevatorAtL4` (true at L3/L4/Barge) – used by scoring logic.
  - `runFunnelIntake` – enables funnel during some sequences.
- Wrist/arm wiring exists but is currently commented out in code.

### EndE (Coral Intake)
- Spark Flex motor on `ArmConstants.IntakeCanID = 18`.
- LaserCan (`CAN ID 29`) detects coral: `isCoralEngaged()` when distance ≤ 85 mm.
- Commands: `runIntakeCommand()`, `reverseIntakeCommand()`, and feeder variants that stop on detection.

### FunnelIntake
- Spark Flex roller on `FunnelConstants.FunnelLIntake = 22`.
- Command: `runIntakeCommand()` spins to move game piece inward; speeds in `FunnelIntakeSetpoints`.

### Climber
- Spark Flex on `ClimbConstants.ClimbMotor = 21`.
- Manual commands: `runClimberCommand()` (in), `runReverseClimberCommand()` (out). Position mode scaffolding present but not used.

### AlgaeArmSubsystem
- Arm Flex motors on `ArmLeftCanID = 17`, `ArmRightCanID = 19` (right follows left per `Configs`).
- Setpoints defined in `Constants.ArmSetpoints` but currently unused by main flows.

## Commands & Alignment
- `LAlignToReefTagRelative` / `RAlignToReefTagRelative`
  - Reads AprilTag-relative pose from Limelight pipelines (`limelight-right` and `limelight-left`).
  - PID on X (depth), Y (lateral), and rotation (yaw) to drive to setpoint:
    - Gains in `Constants`: `X_REEF_ALIGNMENT_P = 0.8`, `Y_REEF_ALIGNMENT_P = 1.5`, `ROT_REEF_ALIGNMENT_P = 0.05`.
    - Setpoints/tolerances in `Constants`: X/Y/ROT positions and tolerances.
  - Finishes when pose held stable or vision lost (`POSE_VALIDATION_TIME`, `DONT_SEE_TAG_WAIT_TIME`).

## Controls (RobotContainer)
- Driver (`Xbox` on port 0)
  - Left stick: translation (X/Y) with deadband `0.13`.
  - Right stick X: rotation. Right stick XY can directly command heading in an alternate default.
  - Right trigger (hold): slow mode (scales translation to 20%, caps rotation to +/-0.5 when above threshold).
  - Right bumper (hold): Reef alignment + score sequence on Right camera.
  - Left bumper (hold): Reef alignment on Left camera.
  - Start: set initial heading to 180° (useful for Red side practice).
  - A/B: Climber manual out/in.
- Operator (`Xbox` on port 1)
  - A/B/X/Y: Elevator to L1/L2/L3/L4 (delayed execution to coordinate sequencing).
  - D-Pad Up/Down: Algae High/Low.
  - D-Pad Left: Barge.
  - Back: FeederStation.
  - Left bumper (hold): Intake coral until LaserCan detects, then brief reverse to settle.

## Autos & PathPlanner
- Autos live in `src/main/deploy/pathplanner/autos`. Paths in `.../paths` and `navgrid.json` for pathfinding.
- `AutoBuilder.buildAutoChooser()` exposes a SmartDashboard chooser: pick your auto from the GUI-defined list.
- NamedCommands registered in `RobotContainer` allow PathPlanner events to trigger:
  - Examples: `Stop`, `test`, `outtake`, `fintake`, setpoints like `ElevatorLiftL1`, `AlgaeHigh`, `Gyroreset`.
- `SwerveSubsystem.getAutonomousCommand(String)` supports building autos by name when needed.

## Constants & Configs
- `Constants`
  - Ports: `OIConstants.kDriverControllerPort = 0`, `kOperatorControllerPort = 1`.
  - Deadbands, trigger threshold.
  - Alignment PID gains and tolerances.
  - Elevator/Climber/Intake setpoint scalars for manual commands.
  - Drive kinematics dimensions used for odometry (not swerve CAN IDs).
- `Configs`
  - Spark Flex/Max configurations, e.g. IdleMode, current limits, voltage compensation.
  - MAXMotion for elevator/arm/wrist (velocity, acceleration, allowed error).
  - Swerve module driving/turning conversion factors and PID for turning.

## Tuning & Calibration
- Swerve (YAGSL)
  - Ensure each module JSON has correct absolute encoder offsets and motor IDs.
  - Verify wheel diameter and gear reductions in `modules/physicalproperties.json` and `pidfproperties.json`.
  - Use module telemetry on SmartDashboard (`module i` angle) to confirm steering directions.
- Limelight
  - Pipelines named `limelight-left` and `limelight-right` are expected by alignment commands.
  - Tune alignment gains in `Constants` if approach is oscillatory or sluggish.
- Elevator/Arm/Intake
  - Adjust MAXMotion velocity/acceleration and PID in `Configs` for your mechanism.
  - Use `setSetpointDelayed` when you need sequencing (elevator first, then wrist/arm if re-enabled).

## Build, Deploy, and Run (WPILib Palette)
- Open Command Palette (`Ctrl+Shift+P`) and use:
  - `WPILib: Set Team Number` (once per machine).
  - `WPILib: Build Robot Code` to compile.
  - `WPILib: Deploy Robot Code` to deploy to the roboRIO.
  - `WPILib: Simulate Robot Code` for desktop simulation.
- Logs: Check the VS Code `Terminal` panel for WPILib/GradleRIO task output.
- Driver Station
  - Select the desired auto from SmartDashboard "Auto Chooser".
  - Field-oriented: ensure heading is set appropriate for alliance before driving (Start button mapped).

## Troubleshooting
- Robot drives the wrong way field-oriented: zero or set heading (Start button), verify navX orientation.
- Swerve wheels twitch or steer incorrectly: check absolute encoder offsets and motor/controller CAN IDs in YAGSL JSONs.
- Reef alignment doesn’t move: verify Limelight pipeline names, tags visible, and correct camera mounting orientation.
- Elevator stalling/overshooting: lower MAXMotion velocity/accel, re-tune P/D in `Configs`.

## Contributing
- Keep changes focused and match code style.
- Update `README.md` and/or inline Javadoc for new subsystems or commands.
- If you add a new mechanism:
  - Put hardware IDs in `Constants`, configs in `Configs`, logic in `subsystems/`, and bind commands in `RobotContainer`.
