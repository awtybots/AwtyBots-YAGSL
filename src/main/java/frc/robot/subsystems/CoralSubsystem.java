package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotController;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
// import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.Constants.IntakeSetpoints;
// import frc.robot.Constants.WristSetpoints;
import au.grapplerobotics.LaserCan;

public class CoralSubsystem extends SubsystemBase {

    public enum Setpoint {
        FeederStation,
        L1,
        L2,
        L3,
        L4,
        AlgaeLow,
        AlgaeHigh,
        Barge;
    }

    // Variable use for tracking if the elevator was raised to L4
    public static boolean ElevatorAtL4;

    public static boolean runFunnelIntake;
    public Setpoint lastSetpoint = Setpoint.FeederStation;

    // Track current setpoint as an enum for other subsystems to reference safely
    public static Setpoint coralCurrentSetpoint = Setpoint.FeederStation;

    // arm setup
    // private SparkFlex r_armMotor = new SparkFlex(ArmConstants.ArmRightCanID,
    // MotorType.kBrushless);
    // private SparkFlex l_armMotor = new SparkFlex(ArmConstants.ArmLeftCanID,
    // MotorType.kBrushless);
    // //private SparkClosedLoopController r_armController =
    // r_armMotor.getClosedLoopController();
    // private SparkClosedLoopController l_armController =
    // l_armMotor.getClosedLoopController();

    // private RelativeEncoder armEncoder = l_armMotor.getEncoder();
    // private AbsoluteEncoder armEncoder = l_armMotor.getAbsoluteEncoder();

    // elevator setup
    private SparkFlex l_elevatorMotor = new SparkFlex(ElevatorConstants.LeftElevatorCanID, MotorType.kBrushless);
    private SparkFlex r_elevatorMotor = new SparkFlex(ElevatorConstants.RightElevatorCanID, MotorType.kBrushless);
    private SparkClosedLoopController l_elevatorController = l_elevatorMotor.getClosedLoopController();
    // private SparkClosedLoopController r_elevatorController =
    // r_elevatorMotor.getClosedLoopController();
    private RelativeEncoder elevatorEncoder = r_elevatorMotor.getEncoder();

    // arm setup
    // private SparkFlex l_armMotor = new SparkFlex(ArmConstants.ArmLeftCanID,
    // MotorType.kBrushless);
    // private SparkFlex r_armMotor = new SparkFlex(ArmConstants.ArmRightCanID,
    // MotorType.kBrushless);
    // private SparkFlex intakeMotor = new SparkFlex(ArmConstants.IntakeCanID,
    // MotorType.kBrushless);
    // private AbsoluteEncoder armEncoder = l_armMotor.getAbsoluteEncoder();

    /*
     * // wrist setup
     * private SparkFlex wristMotor = new SparkFlex(ArmConstants.WristCanID,
     * MotorType.kBrushless);
     * private SparkClosedLoopController wristController =
     * wristMotor.getClosedLoopController();
     * // private RelativeEncoder wristEncoder = wristMotor.getEncoder();
     * private AbsoluteEncoder wristEncoder = wristMotor.getAbsoluteEncoder();
     */

    private boolean wasReset = false;
    // private double armCurrentTarget = ArmSetpoints.FeederStation;
    // private double wristCurrentTarget = WristSetpoints.FeederStation;
    public static double elevatorCurrentTarget = ElevatorSetpoints.FeederStation;
    private boolean wristExecutionEnabled = true;
    private boolean elevatorExecutionEnabled = true;
    // private double wristPendingTarget = WristSetpoints.FeederStation;
    private double elevatorPendingTarget = ElevatorSetpoints.FeederStation;

    public CoralSubsystem() {
        ElevatorAtL4 = false;
        runFunnelIntake = false;

        l_elevatorMotor.configure(
                Configs.CoralSubsystem.l_elevatorMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        r_elevatorMotor.configure(
                Configs.CoralSubsystem.r_elevatorMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        elevatorEncoder.setPosition(0);
        // armEncoder.setPosition(0);

    }

    private void moveToSetpoint() {
        // l_elevatorController.setReference(elevatorCurrentTarget,
        // ControlType.kMAXMotionPositionControl);
        // wristController.setReference(wristCurrentTarget,
        // ControlType.kMAXMotionPositionControl);

        if (elevatorExecutionEnabled) {
            l_elevatorController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
        }
        // if (wristExecutionEnabled) {
        // wristController.setReference(wristCurrentTarget,
        // ControlType.kMAXMotionPositionControl);
        // }

        // if(runFunnelIntake){
        // double elevatorPos = elevatorEncoder.getPosition();
        // double elevatorError = Math.abs(elevatorCurrentTarget - elevatorPos);
        // double stopThreshold = 20;

        // if (elevatorError > stopThreshold){

        // return;
        // }

        // }

        // l_armController.setReference(armCurrentTarget,
        // ControlType.kMAXMotionPositionControl);

    }

    // public Command manualElevatorDown() {
    // return Commands.startEnd(
    // () -> {
    // l_elevatorMotor.set(0.5);
    // r_elevatorMotor.set(-0.5);
    // },
    // () -> {
    // l_elevatorMotor.set(0);
    // r_elevatorMotor.set(0);
    // });
    // }

    /** Zero the arm encoder when the user button is pressed on the roboRIO */
    private void zeroOnUserButton() {
        if (!wasReset && RobotController.getUserButton()) {
            // Zero the encoder only when button switches from "unpressed" to "pressed" to
            // prevent
            // constant zeroing while pressed
            wasReset = true;
            elevatorEncoder.setPosition(0);
        } else if (!RobotController.getUserButton()) {
            wasReset = false;
        }
    }

    public Command resetElevatorEncoder() {
        return this.runOnce(() -> {
            elevatorEncoder.setPosition(0);
        });
    }

    public Command setSetpointCommand(Setpoint setpoint, boolean executeElevator, boolean executeWrist) {
        return this.runOnce(() -> {
            switch (setpoint) {
                case FeederStation:
                    runFunnelIntake = true;
                    ElevatorAtL4 = false;

                    // if (executeElevator) {
                    elevatorCurrentTarget = ElevatorSetpoints.FeederStation;
                    elevatorExecutionEnabled = true;
                    // } else {
                    // elevatorPendingTarget = ElevatorSetpoints.FeederStation;
                    // elevatorExecutionEnabled = false;
                    // }

                    // if (executeWrist) {
                    // wristCurrentTarget = WristSetpoints.FeederStation;
                    // wristExecutionEnabled = true;
                    // } else {
                    // wristPendingTarget = WristSetpoints.FeederStation;
                    // wristExecutionEnabled = false;
                    // }
                    break;

                case L1:
                    ElevatorAtL4 = false;
                    runFunnelIntake = false;

                    // if (executeElevator) {
                    elevatorCurrentTarget = ElevatorSetpoints.L1;
                    elevatorExecutionEnabled = true;
                    // } else {
                    // elevatorPendingTarget = ElevatorSetpoints.L1;
                    // elevatorExecutionEnabled = false;
                    // }

                    // if (executeWrist) {
                    // wristCurrentTarget = WristSetpoints.L1;
                    // wristExecutionEnabled = true;
                    // } else {
                    // wristPendingTarget = WristSetpoints.L1;
                    // wristExecutionEnabled = false;
                    // }
                    break;

                case L2:
                    ElevatorAtL4 = false;
                    runFunnelIntake = false;

                    // if (executeElevator) {
                    elevatorCurrentTarget = ElevatorSetpoints.L2;
                    elevatorExecutionEnabled = true;
                    // } else {
                    // elevatorPendingTarget = ElevatorSetpoints.L2;
                    // elevatorExecutionEnabled = false;
                    // }

                    // if (executeWrist) {
                    // wristCurrentTarget = WristSetpoints.L2;
                    // wristExecutionEnabled = true;
                    // } else {
                    // wristPendingTarget = WristSetpoints.L2;
                    // wristExecutionEnabled = false;
                    // }
                    break;

                case L3:
                    ElevatorAtL4 = true;
                    runFunnelIntake = false;

                    // if (executeElevator) {
                    elevatorCurrentTarget = ElevatorSetpoints.L3;
                    elevatorExecutionEnabled = true;
                    // } else {
                    // elevatorPendingTarget = ElevatorSetpoints.L3;
                    // elevatorExecutionEnabled = false;
                    // }

                    // if (executeWrist) {
                    // wristCurrentTarget = WristSetpoints.L3;
                    // wristExecutionEnabled = true;
                    // } else {
                    // wristPendingTarget = WristSetpoints.L3;
                    // wristExecutionEnabled = false;
                    // }
                    break;

                case L4:
                    ElevatorAtL4 = true;
                    runFunnelIntake = false;

                    // if (executeElevator) {
                    elevatorCurrentTarget = ElevatorSetpoints.L4;
                    elevatorExecutionEnabled = true;
                    // } else {
                    // elevatorPendingTarget = ElevatorSetpoints.L4;
                    // elevatorExecutionEnabled = false;
                    // }

                    // if (executeWrist) {
                    // wristCurrentTarget = WristSetpoints.L4;
                    // wristExecutionEnabled = true;
                    // } else {
                    // wristPendingTarget = WristSetpoints.L4;
                    // wristExecutionEnabled = false;
                    // }
                    break;

                case AlgaeLow:
                    runFunnelIntake = false;
                    ElevatorAtL4 = false;

                    // if (executeElevator) {
                    elevatorCurrentTarget = ElevatorSetpoints.AlgaeLow;
                    elevatorExecutionEnabled = true;
                    // } else {
                    // elevatorPendingTarget = ElevatorSetpoints.AlgaeLow;
                    // elevatorExecutionEnabled = false;
                    // }

                    // if (executeWrist) {
                    // wristCurrentTarget = WristSetpoints.AlgaeLow;
                    // wristExecutionEnabled = true;
                    // } else {
                    // wristPendingTarget = WristSetpoints.AlgaeLow;
                    // wristExecutionEnabled = false;
                    // }
                    break;

                case AlgaeHigh:
                    runFunnelIntake = false;
                    ElevatorAtL4 = false;

                    // if (executeElevator) {
                    elevatorCurrentTarget = ElevatorSetpoints.AlgaeHigh;
                    elevatorExecutionEnabled = true;
                    // } else {
                    // elevatorPendingTarget = ElevatorSetpoints.AlgaeHigh;
                    // elevatorExecutionEnabled = false;
                    // }

                    // if (executeWrist) {
                    // wristCurrentTarget = WristSetpoints.AlgaeHigh;
                    // wristExecutionEnabled = true;
                    // } else {
                    // wristPendingTarget = WristSetpoints.AlgaeHigh;
                    // wristExecutionEnabled = false;
                    // }
                    break;

                case Barge:
                    ElevatorAtL4 = true;
                    runFunnelIntake = false;

                    // if (executeElevator) {
                    elevatorCurrentTarget = ElevatorSetpoints.Barge;
                    elevatorExecutionEnabled = true;
                    // } else {
                    // elevatorPendingTarget = ElevatorSetpoints.Barge;
                    // elevatorExecutionEnabled = false;
                    // }

                    // if (executeWrist) {
                    // wristCurrentTarget = WristSetpoints.Barge;
                    // wristExecutionEnabled = true;
                    // } else {
                    // wristPendingTarget = WristSetpoints.Barge;
                    // wristExecutionEnabled = false;
                    // }
                    break;
            }
            lastSetpoint = setpoint;
            // Ensure the current setpoint enum is always updated
            coralCurrentSetpoint = setpoint;
        });
    }

    /**
     * Original behavior - execute both immediately
     */
    public Command setSetpointCommand(Setpoint setpoint) {
        coralCurrentSetpoint = setpoint;
        return setSetpointCommand(setpoint, true, true);
    }

    /**
     * Delay both elevator and wrist execution
     */
    public Command setSetpointDelayed(Setpoint setpoint) {
        coralCurrentSetpoint = setpoint;
        return setSetpointCommand(setpoint, false, false);
    }

    /**
     * Execute elevator now, delay wrist
     */
    public Command setSetpointElevatorFirst(Setpoint setpoint) {
        coralCurrentSetpoint = setpoint;
        return setSetpointCommand(setpoint, true, false);
    }

    /**
     * Execute wrist now, delay elevator
     */
    public Command setSetpointWristFirst(Setpoint setpoint) {
        coralCurrentSetpoint = setpoint;
        return setSetpointCommand(setpoint, false, true);
    }

    public void periodic() {
        moveToSetpoint();
        zeroOnUserButton();

        // Display subsystem values

        // SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
        // SmartDashboard.putNumber("Coral/Arm/Actual Position",
        // armEncoder.getPosition());
        // SmartDashboard.putNumber("Coral/Elevator/Target Position",
        // elevatorCurrentTarget);
        // SmartDashboard.putNumber("Coral/Elevator/Actual Position",
        // elevatorEncoder.getPosition());
        // SmartDashboard.putNumber("Coral/Wrist/Target Position",
        // wristAbsoluteEncoder.getPosition());

        // SmartDashboard.putNumber("Coral/Arm/Target Position",
        // armEncoder.getPosition());

        // System.out.println("Wrist Encoder Position: " + wristEncoder.getPosition());
        // System.out.println("LaserCan value: " + measurement.status);
        // System.out.println("Normal Arm Position: " + armEncoder.getPosition()+ "
        // Absolute Arm Position: ");
        // SmartDashboard.putNumber("Coral/Intake/Applied Output",
        // intakeMotor.getAppliedOutput();
    }
}