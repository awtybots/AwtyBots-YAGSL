package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import frc.robot.subsystems.CoralSubsystem;
import frc.robot.subsystems.Algae.Setpoint;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.Constants.ArmSetpoints;

public class AlgaeArmSubsystem extends SubsystemBase {

    public enum AlgaeSetpoint {
        GroundIntake,
        Reef,
        Stow,
        Barge,
    }

    private SparkFlex l_armMotor = new SparkFlex(ArmConstants.ArmLeftCanID, MotorType.kBrushless);
    private SparkFlex r_armMotor = new SparkFlex(ArmConstants.ArmRightCanID, MotorType.kBrushless);
    private SparkClosedLoopController r_armController = r_armMotor.getClosedLoopController();
    private SparkClosedLoopController l_armController = l_armMotor.getClosedLoopController();

    private AbsoluteEncoder armEncoder = l_armMotor.getAbsoluteEncoder();

    private double armCurrentTarget;
    private final CoralSubsystem coralSubsystem;

    // private boolean armExecutionEnabled = true;
    // private double armPendingTarget = ArmSetpoints.Stow;

    public AlgaeArmSubsystem(CoralSubsystem coralSubsystem) {
        this.coralSubsystem = coralSubsystem;

        l_armMotor.configure(
                Configs.CoralSubsystem.l_armMotorConfig, // update configs
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        r_armMotor.configure(
                Configs.CoralSubsystem.r_armMotorConfig, // update configs
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);

        // Initialize target to current arm position to avoid motion on boot
        armCurrentTarget = ArmSetpoints.Stow;
    }

    private void moveToSetpoint() {
        // l_elevatorController.setReference(elevatorCurrentTarget,
        // ControlType.kMAXMotionPositionControl);
        // wristController.setReference(wristCurrentTarget,
        // ControlType.kMAXMotionPositionControl);

        // if (CoralSubsystem.elevatorCurrentTarget == ElevatorSetpoints.FeederStation)
        // {
        l_armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
        // }

    }

    public Command coralToAlgae() {
        return this.runOnce(() -> {
            // Use the enum setpoint rather than elevator target doubles,
            // since several elevator setpoints share the same numeric value (e.g. 0.0)
            switch (CoralSubsystem.coralCurrentSetpoint) {
                case FeederStation:
                    armCurrentTarget = ArmSetpoints.Stow;
                    break;
                case L1:
                    break;
                case L2:
                    if (coralSubsystem.isElevatorProgressAt(0.40)) {
                        armCurrentTarget = ArmSetpoints.AlgaeIntake;
                    }
                    break;
                case L3:
                    break;
                case L4:
                    // Preserve previous behavior mapping L4 to barge position
                    if (coralSubsystem.isElevatorProgressAt(0.40)) {
                        armCurrentTarget = ArmSetpoints.Barge;
                    }
                    break;
                case AlgaeLow:
                    break;
                case AlgaeHigh:
                    if (coralSubsystem.isElevatorProgressAt(0.40)) {
                        armCurrentTarget = ArmSetpoints.AlgaeIntake;
                    }
                    break;
                case Barge:
                    if (coralSubsystem.isElevatorProgressAt(.05)) {
                        armCurrentTarget = ArmSetpoints.Barge;
                    }

                    break;
                default:
                    // No change for unspecified cases
                    break;
            }
        });
    }

    // Return a no-op command as a fallback

    public Command setSetpointCommand(AlgaeSetpoint setpoint) {
        return this.runOnce(() -> {
            switch (setpoint) {

                case GroundIntake:
                    armCurrentTarget = ArmSetpoints.GroundIntake;
                    // armExecutionEnabled = true;
                    break;

                case Reef: // intaking from reef
                    armCurrentTarget = ArmSetpoints.AlgaeIntake;
                    // armExecutionEnabled = true;
                    break;

                case Stow:
                    armCurrentTarget = ArmSetpoints.Stow;
                    // armExecutionEnabled = true;
                    break;

                case Barge:
                    armCurrentTarget = ArmSetpoints.Barge;
                    // armExecutionEnabled = true;
                    break;

            }
        });
    }

    // if (CoralSubsystem.elevatorCurrentTarget == ElevatorSetpoints.FeederStation)

    public void periodic() {
        moveToSetpoint();
    }

}
