// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.LAlignToReefTagRelative;
import frc.robot.commands.RAlignToReefTagRelative;
import frc.robot.Constants.IntakeSetpoints;
import frc.robot.Constants.OIConstants;
//import frc.robot.commands.Autos;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.EndE;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.FunnelIntake;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;
// import frc.robot.subsystems.Algae;
// import frc.robot.subsystems.AlgaeArmSubsystem;

import java.io.File;
import java.util.Map;

import com.ctre.phoenix6.hardware.core.CoreCANcoder;
import com.ctre.phoenix6.mechanisms.swerve.LegacySwerveRequest.RobotCentric;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final EndE m_EndE = new EndE();
  private final FunnelIntake m_funnelIntakeSubsystem = new FunnelIntake();
  private final Climber m_climber = new Climber();
  private final SendableChooser<Command> autoChooser;
  // private final Algae m_algae = new Algae();
  // private final AlgaeArmSubsystem m_AlgaeArmSubsystem = new AlgaeArmSubsystem(m_coralSubsystem);
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController = new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController = new CommandXboxController(
      OIConstants.kOperatorControllerPort);

  // Driver controller triggers
  private final Trigger slowModeTrigger = m_driverController.rightTrigger(OIConstants.kTriggerThreshold);
  private final Trigger driverHeadingResetTrigger = m_driverController.start();
  private final Trigger driverAlignRightTrigger = m_driverController.rightBumper();
  private final Trigger driverAlignLeftTrigger = m_driverController.leftBumper();
  private final Trigger driverClimberInTrigger = m_driverController.b();
  private final Trigger driverClimberOutTrigger = m_driverController.a();

  // Operator controller triggers
  private final Trigger operatorIntakeTrigger = m_operatorController.leftBumper();
  private final Trigger operatorReverseIntakeTrigger = m_operatorController.rightBumper();
  private final Trigger operatorFeederStationTrigger = m_operatorController.back();
  private final Trigger operatorL1Trigger = m_operatorController.a();
  private final Trigger operatorL2Trigger = m_operatorController.b();
  private final Trigger operatorL3Trigger = m_operatorController.x();
  private final Trigger operatorL4Trigger = m_operatorController.y();
  private final Trigger operatorAlgaeHighTrigger = m_operatorController.povUp();
  private final Trigger operatorAlgaeLowTrigger = m_operatorController.povDown();
  private final Trigger operatorBargeTrigger = m_operatorController.povLeft();

  // Sensor/state-based triggers
  private final Trigger coralDetectedTrigger = new Trigger(m_EndE::isCoralEngaged);
  private final Trigger coralLostTrigger = coralDetectedTrigger.negate();
  private final Trigger elevatorAtL4Trigger = new Trigger(() -> CoralSubsystem.ElevatorAtL4);
  private final Trigger funnelIntakeTrigger = new Trigger(() -> CoralSubsystem.runFunnelIntake);


  public static boolean hasLostContact = false;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer() {
    // Default to 0° (assuming forward should be field-oriented default)
    double startingAngle = 0;

    // var alliance = DriverStation.getAlliance();

    // if (alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red) {
    // // If on Red Alliance, adjust heading to 180°
    // startingAngle = 180;
    // } else if (!DriverStation.isFMSAttached() && !DriverStation.isDSAttached()) {
    // // If NOT connected to FMS or Driver Station (testing mode), allow manual
    // // setting
    // startingAngle = 0;
    // System.out.println("Practice Mode: Setting starting heading to " +
    // startingAngle);
    // }

    // Set the correct initial heading for field-oriented driving
    drivebase.setInitialHeading(startingAngle);

    // Configure the trigger bindings
    drivebase.setDefaultCommand(driveFieldOrientedAngluarVelocity);
    NamedCommands.registerCommand("Stop", Commands.runOnce(() -> drivebase.stop()));
    NamedCommands.registerCommand("test", Commands.print("Hello World"));
    NamedCommands.registerCommand("outtake", m_EndE.reverseIntakeCommand().withTimeout(1));
    NamedCommands.registerCommand("outtakeLD", m_EndE.BrunIntakeCommandFeeder().andThen(m_EndE.runIntakeCommand().withTimeout(.2)));
    NamedCommands.registerCommand("outtakefast", m_EndE.fastrunIntakeCommand().withTimeout(0.3));
    NamedCommands.registerCommand("outtake0.5", m_EndE.reverseIntakeCommand().withTimeout(0.5));
    NamedCommands.registerCommand("fintake", m_funnelIntakeSubsystem.runIntakeCommand().withTimeout(1));
    NamedCommands.registerCommand("FeederStation", m_coralSubsystem.setSetpointCommand(Setpoint.FeederStation));
    NamedCommands.registerCommand("ElevatorLiftL1", m_coralSubsystem.setSetpointCommand(Setpoint.L1));
    NamedCommands.registerCommand("ElevatorLiftL2", m_coralSubsystem.setSetpointCommand(Setpoint.L2));
    NamedCommands.registerCommand("ElevatorLiftL3", m_coralSubsystem.setSetpointCommand(Setpoint.L3));
    NamedCommands.registerCommand("ElevatorLiftL4", m_coralSubsystem.setSetpointCommand(Setpoint.L4));
    NamedCommands.registerCommand("AlgaeLow", m_coralSubsystem.setSetpointCommand(Setpoint.AlgaeLow));
    NamedCommands.registerCommand("AlgaeHigh", m_coralSubsystem.setSetpointCommand(Setpoint.AlgaeHigh));
    NamedCommands.registerCommand("Gyroreset", new InstantCommand(() -> drivebase.setInitialHeading(180), drivebase));
    NamedCommands.registerCommand("Gyroreset1", new InstantCommand(() -> drivebase.setInitialHeading(0), drivebase));
    // NamedCommands.registerCommand("AlignR", new SequentialCommandGroup(
    //     Commands.waitUntil(() -> m_EndE.isCoralEngaged()),
    //     m_coralSubsystem.setSetpointCommand(Setpoint.L4),
    //     new RAlignToReefTagRelative(drivebase), this.ScoreUniversal().withTimeout(1)));
    // NamedCommands.registerCommand("AlignL", new SequentialCommandGroup(
    //     Commands.waitUntil(() -> m_EndE.isCoralEngaged()),
    //     m_coralSubsystem.setSetpointCommand(Setpoint.L4),
    //     new LAlignToReefTagRelative(drivebase), this.ScoreUniversal().withTimeout(1)));
    NamedCommands.registerCommand("AlignR",
    new RAlignToReefTagRelative(drivebase));
NamedCommands.registerCommand("AlignL1",
    new LAlignToReefTagRelative(drivebase));
    NamedCommands.registerCommand("AlignRFeeder",
        new RAlignToReefTagRelative(drivebase).withTimeout(5));
    NamedCommands.registerCommand("AlignL",
        new LAlignToReefTagRelative(drivebase).withTimeout(3));
        NamedCommands.registerCommand("AlignLFeeder",
        new LAlignToReefTagRelative(drivebase).withTimeout(5));
    autoChooser = AutoBuilder.buildAutoChooser();


    SmartDashboard.putData("Auto Chooser", autoChooser);

    DriverStation.silenceJoystickConnectionWarning(true);

    configureBindings();
  }

  SwerveInputStream driveAngulareVelocity = SwerveInputStream.of(
      drivebase.getSwerveDrive(),
      () -> m_driverController.getLeftY() * 1,
      () -> m_driverController.getLeftX() * 1)
      .withControllerRotationAxis(m_driverController::getRightX)
      .deadband(OIConstants.DEADBAND)
      .scaleTranslation(0.8)
      .allianceRelativeControl(true);

  SwerveInputStream driveDirectAngle = driveAngulareVelocity
      .copy()
      .withControllerHeadingAxis(m_driverController::getRightY, m_driverController::getRightX)
      .headingWhile(false);

  Command driveFieldOrietedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngluarVelocity = drivebase.driveFieldOriented(driveAngulareVelocity);

  // Command ScoreUniversal() {
  //   return Commands.either(
  //       Commands.either(
  //           Commands.parallel(
  //               m_funnelIntakeSubsystem.runIntakeCommand(), // Run Funnel Intake
  //               m_EndE.runIntakeCommand() // Run Coral Intake at the same time
  //           ),
  //           Commands.either(
  //               m_EndE.runIntakeCommand(), // If ElevatorAtL4 is true, run Reverse Intake
  //               m_EndE.runIntakeCommand(), // Otherwise, run normal intake
  //               () -> CoralSubsystem.ElevatorAtL4 // Condition for reverse intake
  //           ),
  //           () -> CoralSubsystem.runFunnelIntake // Condition for Funnel Intake
  //       ),
  //       m_EndE.runIntakeCommand(), // Do nothing
  //       () -> CoralSubsystem.runFunnelIntake || CoralSubsystem.ElevatorAtL4);
  // }

  Command ScoreUniversal() {
    return Commands.either(
        m_EndE.reverseIntakeCommand(), // Run reverse intake at L4 or L3
        m_EndE.runIntakeCommand(),        // Run normal intake everywhere else
        () -> CoralSubsystem.ElevatorAtL4);
}
Command Intake() {
  return Commands.either(
      m_EndE.reverseIntakeCommand(), // Run reverse intake at L4 or L3
      m_EndE.BrunIntakeCommandFeeder().andThen(m_EndE.runIntakeCommand().withTimeout(.2)),        // Run normal intake everywhere else
      () -> CoralSubsystem.ElevatorAtL4);
}

Command Universal() {
  if (CoralSubsystem.runFunnelIntake) {
      return m_EndE.BrunIntakeCommandFeeder()
                   .andThen(m_EndE.runIntakeCommand().withTimeout(0.2));
  } 
  
  if (CoralSubsystem.ElevatorAtL4) {
      return m_EndE.reverseIntakeCommand();
  }

  return m_EndE.runIntakeCommand();
}
Command scoreUniversal() {
    return Commands.select(Map.of(
        "FUNNEL",   m_EndE.BrunIntakeCommandFeeder()
                         .andThen(m_EndE.runIntakeCommand().withTimeout(0.2)),
        "L4",       m_EndE.reverseIntakeCommand(),
        "DEFAULT",  m_EndE.runIntakeCommand()
    ), () -> {
        if (CoralSubsystem.runFunnelIntake) return "FUNNEL";
        if (CoralSubsystem.ElevatorAtL4)    return "L4";
        return "DEFAULT";
    });
}


  /**
   * Use this method to define your trigger->command mappings. Triggers can be
   * created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with
   * an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link
   * CommandXboxController
   * Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or
   * {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    // enable slow mode
    slowModeTrigger
        .onTrue(
            Commands.runOnce(() -> {
              driveAngulareVelocity.scaleTranslation(0.2);
              driveAngulareVelocity.withControllerRotationAxis(() -> {
                double rotationValue = m_driverController.getRightX();
                return Math.abs(rotationValue) >= 0.5 ? 0.5 * Math.signum(rotationValue) : rotationValue;
              });
            }))
        .onFalse(
            Commands.runOnce(() -> {
              driveAngulareVelocity.scaleTranslation(1.0);
              driveAngulareVelocity.withControllerRotationAxis(m_driverController::getRightX);
            }));

    // Left Bumper -> Run tube intake
    operatorIntakeTrigger
        .and(funnelIntakeTrigger)
        .whileTrue(m_EndE.BrunIntakeCommandFeeder().andThen(m_EndE.runIntakeCommand().withTimeout(.2)));
     //   .whileTrue(m_EndE.BrunIntakeCommandFeeder(hasLostContact).andThen(m_EndE.reverseIntakeCommand().withTimeout(.2)));// .onlyIf(m_operatorController.leftBumper()));

    // m_operatorController.leftBumper().whileTrue(new CoralIntake(m_EndE));
    // m_operatorController.start().whileTrue(m_coralSubsystem.manualElevatorDown());

    operatorIntakeTrigger
        .and(funnelIntakeTrigger.negate())
        .and(elevatorAtL4Trigger)
        .whileTrue(m_EndE.reverseIntakeCommand());

    operatorIntakeTrigger
        .and(funnelIntakeTrigger.negate())
        .and(elevatorAtL4Trigger.negate())
        .whileTrue(m_EndE.runIntakeCommand());

    // Right Bumper -> Run tube intake in reverse
    // m_driverController.y().whileTrue(m_algae.runAlgaeInCommand());
    operatorReverseIntakeTrigger.whileTrue(m_EndE.reverseIntakeCommand());

    // Reef alignment
    // m_driverController.rightBumper().whileTrue(new
    // RAlignToReefTagRelative(drivebase));
    // m_driverController.leftBumper().whileTrue(new
    // LAlignToReefTagRelative(drivebase));
    driverAlignRightTrigger.whileTrue(new RAlignToReefTagRelative(drivebase));
    driverAlignLeftTrigger.whileTrue(new LAlignToReefTagRelative(drivebase));

    // B Button -> Elevator/Arm to human player position, set ball intake to stow when idle
    operatorFeederStationTrigger.onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.FeederStation));
    // A Button -> Elevator/Arm to level 1 position
    operatorL1Trigger.onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.L1));

    // B Button -> Elevator/Arm to level 2 position
    operatorL2Trigger.onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.L2));

    // X Button -> Elevator/Arm to level 3 position
    operatorL3Trigger.onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.L3));

    // Y Button -> Elevator/Arm to level 4 position
    operatorL4Trigger.onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.L4));

    // D-Pad Up -> Elevator to 2st Algae pickup position
    operatorAlgaeHighTrigger.onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.AlgaeHigh));

    // D-Pad Down -> Elevator to 1st Algae pickup position
    operatorAlgaeLowTrigger.onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.AlgaeLow));
    // D-Pad Left -> Elevator to Barge position
    operatorBargeTrigger.onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.Barge));

    driverHeadingResetTrigger.onTrue(new InstantCommand(() -> drivebase.setInitialHeading(180), drivebase));
    // m_driverController.rightTrigger().whileTrue(this.ScoreUniversal());
    // m_operatorController.rightStick().onTrue(m_coralSubsystem.resetElevatorEncoder());

    // A Button -> Climber Goes In
    driverClimberInTrigger.whileTrue(m_climber.runClimberCommand());
    // B Button -> Climber Goes Out
    driverClimberOutTrigger.whileTrue(m_climber.runReverseClimberCommand());

    // Track coral contact to avoid rescheduling intake commands when contact is lost/regained.
    coralDetectedTrigger.onTrue(Commands.runOnce(() -> hasLostContact = false));
    coralLostTrigger.debounce(0.1).onTrue(Commands.runOnce(() -> hasLostContact = true));

    // Resets all encoders
    // m_operatorController.start().onTrue(m_coralSubsystem.resetAllEncoders());
    // m_operatorController.leftTrigger(0.3).whileTrue(m_AlgaeArmSubsystem.coralToAlgae());
    // m_operatorController.leftTrigger(0.5).whileTrue((m_AlgaeArmSubsystem.coralToAlgae()));
    // m_operatorController.rightTrigger(0.5).whileTrue((m_AlgaeArmSubsystem.runGroundArmAlgaeCommand().alongWith(m_algae.runAlgaeInCommand())));
    // m_operatorController.povRight().whileTrue(m_algae.runAlgaeOutCommand());
  }

  public PathPlannerAuto pathPlannerAuto() {
    return new PathPlannerAuto("Vision3CLeft", true);
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}
