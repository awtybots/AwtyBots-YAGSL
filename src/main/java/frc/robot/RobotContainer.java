// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.AutoIntake;
import frc.robot.commands.Autos;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.CoralSubsystem.Setpoint;
import frc.robot.subsystems.FunnelIntake;
import frc.robot.subsystems.IntakeCoralSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveInputStream;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final CoralSubsystem m_coralSubsystem = new CoralSubsystem();
  private final FunnelIntake m_funnelIntakeSubsystem = new FunnelIntake();
  private final IntakeCoralSubsystem intake = new IntakeCoralSubsystem();
  private final SendableChooser<Command> autoChooser;
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OIConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OIConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    drivebase.setDefaultCommand(driveFieldOrientedAngluarVelocity);
    NamedCommands.registerCommand("test",Commands.print("Hello World"));
    NamedCommands.registerCommand("intake", new AutoIntake(intake));
    NamedCommands.registerCommand("outtake", m_funnelIntakeSubsystem.runIntakeCommand());
    NamedCommands.registerCommand("FeederStation", m_coralSubsystem.setSetpointCommand(Setpoint.FeederStation));
    NamedCommands.registerCommand("ElevatorLiftL1", m_coralSubsystem.setSetpointCommand(Setpoint.L1));
    NamedCommands.registerCommand("ElevatorLiftL2", m_coralSubsystem.setSetpointCommand(Setpoint.L2));
    NamedCommands.registerCommand("ElevatorLiftL3", m_coralSubsystem.setSetpointCommand(Setpoint.L3));
    NamedCommands.registerCommand("ElevatorLiftL4", m_coralSubsystem.setSetpointCommand(Setpoint.L4));
    NamedCommands.registerCommand("AlgaeLow", m_coralSubsystem.setSetpointCommand(Setpoint.AlgaeLow));
    NamedCommands.registerCommand("AlgaeHigh", m_coralSubsystem.setSetpointCommand(Setpoint.AlgaeHigh));


    autoChooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", autoChooser);

    DriverStation.silenceJoystickConnectionWarning(true);

    configureBindings();
  }

  SwerveInputStream driveAngulareVelocity =
      SwerveInputStream.of(
              drivebase.getSwerveDrive(),
              () -> m_driverController.getLeftY() * 1,
              () -> m_driverController.getLeftX() * 1)
          .withControllerRotationAxis(m_driverController::getRightX)
          .deadband(OIConstants.DEADBAND)
          .scaleTranslation(0.8)
          .allianceRelativeControl(true);

  

  SwerveInputStream driveDirectAngle =
      driveAngulareVelocity
          .copy()
          .withControllerHeadingAxis(m_driverController::getRightY, m_driverController::getRightX)
          .headingWhile(false);

  Command driveFieldOrietedDirectAngle = drivebase.driveFieldOriented(driveDirectAngle);

  Command driveFieldOrientedAngluarVelocity = drivebase.driveFieldOriented(driveAngulareVelocity);

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {
    //enable slow mode
    m_driverController
  .rightTrigger(OIConstants.kTriggerThreshold)
  .onTrue(
    Commands.runOnce(() -> {driveAngulareVelocity.scaleTranslation(.2);
    }))
    .onFalse(
      Commands.runOnce(() -> {driveAngulareVelocity.scaleTranslation(.8);
    }));

     // Left Bumper -> Run tube intake
     m_operatorController.leftBumper().whileTrue(m_coralSubsystem.reverseIntakeCommand());

     // Right Bumper -> Run tube intake in reverse
     m_operatorController.rightBumper().whileTrue(m_funnelIntakeSubsystem.runIntakeCommand());
     m_operatorController.rightBumper().whileTrue(m_coralSubsystem.runIntakeCommand());
 
     // B Button -> Elevator/Arm to human player position, set ball intake to stow
     // when idle
     m_operatorController.back().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.FeederStation));
 
     // A Button -> Elevator/Arm to level 1 position
     m_operatorController.a().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.L1));
 
     // B Button -> Elevator/Arm to level 2 position
     m_operatorController.b().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.L2));
 
     // X Button -> Elevator/Arm to level 3 position
     m_operatorController.x().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.L3));
 
     // Y Button -> Elevator/Arm to level 4 position
     m_operatorController.y().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.L4));
 
     // D-Pad Up -> Elevator to 2st Algae pickup position
     m_operatorController.povUp().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.AlgaeHigh));
 
     // D-Pad Down -> Elevator to 1st Algae pickup position
     m_operatorController.povDown().onTrue(m_coralSubsystem.setSetpointCommand(Setpoint.AlgaeLow));
 


    //m_driverController.a().onTrue(Commands.runOnce(drivebase::zeroGyro));
    m_driverController.start().onTrue(Commands.runOnce(drivebase::zeroGyro));

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
