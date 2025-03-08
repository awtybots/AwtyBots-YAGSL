package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ArmSetpoints;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.Constants.IntakeSetpoints;
import frc.robot.Constants.WristSetpoints;

public class CoralSubsystem extends SubsystemBase {

    public enum Setpoint {
        FeederStation,
        L1,
        L2,
        L3,
        L4,
        AlgaeLow,
        AlgaeHigh;
    }

    // Variable use for tracking if the elevator was raised to L4
    public static boolean ElevatorAtL4;


    public static boolean runFunnelIntake;

    // arm setup
    private SparkFlex l_armMotor = new SparkFlex(ArmConstants.ArmLeftCanID, MotorType.kBrushless);
    private SparkFlex r_armMotor = new SparkFlex(ArmConstants.ArmRightCanID, MotorType.kBrushless);
    private SparkClosedLoopController l_armController = l_armMotor.getClosedLoopController();
    private SparkClosedLoopController r_armController = r_armMotor.getClosedLoopController();
    private RelativeEncoder armEncoder = l_armMotor.getEncoder();

    // elevator setup
    private SparkFlex l_elevatorMotor = new SparkFlex(ElevatorConstants.LeftElevatorCanID, MotorType.kBrushless);
    private SparkFlex r_elevatorMotor = new SparkFlex(ElevatorConstants.RightElevatorCanID, MotorType.kBrushless);
    private SparkClosedLoopController l_elevatorController = l_elevatorMotor.getClosedLoopController();
    private SparkClosedLoopController r_elevatorController = r_elevatorMotor.getClosedLoopController();
    private RelativeEncoder elevatorEncoder = l_elevatorMotor.getEncoder();

    // wrist setup
    private SparkFlex wristMotor = new SparkFlex(ArmConstants.WristCanID, MotorType.kBrushless);
    private SparkClosedLoopController wristController = wristMotor.getClosedLoopController();
    private RelativeEncoder wristEncoder = wristMotor.getEncoder();

    // intake setup
    private SparkFlex intakeMotor = new SparkFlex(ArmConstants.IntakeCanID, MotorType.kBrushless);

    private boolean wasReset = false;
    private double armCurrentTarget = ArmSetpoints.FeederStation;
    private double wristCurrentTarget = WristSetpoints.FeederStation;
    private double elevatorCurrentTarget = ElevatorSetpoints.FeederStation;

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

        r_armMotor.configure(
                Configs.CoralSubsystem.r_armMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        l_armMotor.configure(
                Configs.CoralSubsystem.l_armMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        wristMotor.configure(
                Configs.CoralSubsystem.wristMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        intakeMotor.configure(
                Configs.CoralSubsystem.intakeMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        armEncoder.setPosition(0);
        elevatorEncoder.setPosition(0);
        wristEncoder.setPosition(0);
    }

    private void moveToSetpoint() {
        l_elevatorController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
        r_elevatorController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
        if(runFunnelIntake){
            double elevatorPos = elevatorEncoder.getPosition();
            double elevatorError = Math.abs(elevatorCurrentTarget - elevatorPos);
            double stopThreshold = 0.5;

            if (elevatorError > stopThreshold){

                return;
            }



        }
        l_armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
        r_armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
        wristController.setReference(wristCurrentTarget, ControlType.kMAXMotionPositionControl);
        
    }

    // public Command manualElevatorDown() {
    //     return Commands.startEnd(
    //             () -> {
    //                 l_elevatorMotor.set(0.5);
    //                 r_elevatorMotor.set(-0.5);
    //             },
    //             () -> {
    //                 l_elevatorMotor.set(0);
    //                 r_elevatorMotor.set(0);
    //             });
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

    private void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce(
                () -> {
                    switch (setpoint) {
                        case FeederStation:
                            runFunnelIntake = true;
                            ElevatorAtL4 = false;
                            armCurrentTarget = ArmSetpoints.FeederStation;
                            wristCurrentTarget = WristSetpoints.FeederStation;
                            elevatorCurrentTarget = ElevatorSetpoints.FeederStation;

                            break;
                        case L1:
                            ElevatorAtL4 = false;
                            runFunnelIntake = false;
                            armCurrentTarget = ArmSetpoints.L1;
                            wristCurrentTarget = WristSetpoints.L1;
                            elevatorCurrentTarget = ElevatorSetpoints.L1;
                            break;
                        case AlgaeLow:
                            runFunnelIntake = false;
                            ElevatorAtL4 = false;
                            elevatorCurrentTarget = ElevatorSetpoints.AlgaeLow;
                            wristCurrentTarget = WristSetpoints.AlgaeLow;
                            armCurrentTarget = ArmSetpoints.AlgaeLow;
                            break;
                        case AlgaeHigh:
                            runFunnelIntake = false;
                            ElevatorAtL4 = false;
                            elevatorCurrentTarget = ElevatorSetpoints.AlgaeHigh;
                            wristCurrentTarget = WristSetpoints.AlgaeHigh;
                            armCurrentTarget = ArmSetpoints.AlgaeHigh;
                            break;
                        case L2:
                            ElevatorAtL4 = false;
                            runFunnelIntake = false;
                            armCurrentTarget = ArmSetpoints.L2;
                            wristCurrentTarget = WristSetpoints.L2;
                            elevatorCurrentTarget = ElevatorSetpoints.L2;
                            break;
                        case L3:
                            ElevatorAtL4 = false;
                            runFunnelIntake = false;
                            armCurrentTarget = ArmSetpoints.L3;
                            wristCurrentTarget = WristSetpoints.L3;
                            elevatorCurrentTarget = ElevatorSetpoints.L3;
                            break;
                        case L4:
                            ElevatorAtL4 = true;
                            runFunnelIntake = false;
                            armCurrentTarget = ArmSetpoints.L4;
                            wristCurrentTarget = WristSetpoints.L4;
                            elevatorCurrentTarget = ElevatorSetpoints.L4;
                            break;
                    }
                });
    }

    public Command runIntakeCommand() {
        return Commands.startEnd(
                () -> setIntakePower(IntakeSetpoints.kForward), () -> setIntakePower(0.0));
    }

    public Command reverseIntakeCommand() {
        return this.startEnd(
                () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
    }

    public void periodic() {
        moveToSetpoint();
        zeroOnUserButton();

        // Display subsystem values

        // SmartDashboard.putNumber("Coral/Arm/Target Position", armCurrentTarget);
        // SmartDashboard.putNumber("Coral/Arm/Actual Position",
        // armEncoder.getPosition());
        SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
        SmartDashboard.putNumber("Coral/Elevator/Actual Position", elevatorEncoder.getPosition());
        // SmartDashboard.putNumber("Coral/Intake/Applied Output",
        // intakeMotor.getAppliedOutput());
    }
}