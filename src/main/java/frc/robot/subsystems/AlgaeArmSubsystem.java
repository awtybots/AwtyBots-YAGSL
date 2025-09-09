package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ElevatorSetpoints;
import frc.robot.Constants.ArmSetpoints;


public class AlgaeArmSubsystem extends SubsystemBase {
    
    public enum Setpoint {
        GroundIntake,
        AlgaeIntake,
        Stow,
        Barge,
        Processor,
    }

    private SparkFlex l_armMotor = new SparkFlex(ArmConstants.ArmLeftCanID, MotorType.kBrushless);
    private SparkFlex r_armMotor = new SparkFlex(ArmConstants.ArmRightCanID, MotorType.kBrushless);
    
    private SparkClosedLoopController l_armController = l_armMotor.getClosedLoopController();
    
    private AbsoluteEncoder armEncoder = l_armMotor.getAbsoluteEncoder();


    private double armCurrentTarget = ArmSetpoints.Stow;
    private boolean armExecutionEnabled = true;
    private double armPendingTarget = ArmSetpoints.Stow;

    public AlgaeArmSubsystem() {

        r_armMotor.configure(
        Configs.CoralSubsystem.r_armMotorConfig,  // update configs
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);

        l_armMotor.configure(
        Configs.CoralSubsystem.l_armMotorConfig,  // update configs
        ResetMode.kResetSafeParameters,
        PersistMode.kPersistParameters);


    }

    private void moveToSetpoint() {
        // l_elevatorController.setReference(elevatorCurrentTarget,
        // ControlType.kMAXMotionPositionControl);
        // wristController.setReference(wristCurrentTarget,
        // ControlType.kMAXMotionPositionControl);

        // if (armExecutionEnabled) {
            l_armController.setReference(armCurrentTarget, ControlType.kMAXMotionPositionControl);
        // }        

    }

    public Command setSetpointCommand(Setpoint setpoint, boolean executeArm) {
        return this.runOnce(() -> {
            switch (setpoint) {

                case GroundIntake:
                    armCurrentTarget = ArmSetpoints.GroundIntake;
                    armExecutionEnabled = true;
                    break;

                case AlgaeIntake:  // intaking from reef
                    armCurrentTarget = ArmSetpoints.AlgaeIntake;
                    armExecutionEnabled = true;
                    break;

                case Stow:
                    armCurrentTarget = ArmSetpoints.Stow;
                    armExecutionEnabled = false;
                    break;

                case Barge:
                    armCurrentTarget = ArmSetpoints.Barge;
                    armExecutionEnabled = true;
                    break;

                case Processor:
                    armCurrentTarget = ArmSetpoints.Processor;
                    armExecutionEnabled = true;
                    break;

            }
        });
    }

    public Command setSetpointCommand(Setpoint setpoint) {  // it has the same name but somehow is recognized as different because of the # of args???
        return setSetpointCommand(setpoint, true);
    }

    public void periodic() {
        moveToSetpoint();
    }


}
