package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import au.grapplerobotics.CanBridge;
import au.grapplerobotics.LaserCan;
import au.grapplerobotics.interfaces.LaserCanInterface.Measurement;
import au.grapplerobotics.ConfigurationFailedException;
import java.time.Period;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import au.grapplerobotics.GrappleJNI;
//import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeSetpoints;
import frc.robot.Robot;

public class EndE {
    private LaserCan lc = new LaserCan(29);
    // intake setup
    private SparkFlex intakeMotor = new SparkFlex(ArmConstants.IntakeCanID, MotorType.kBrushless);

    public EndE() {
        intakeMotor.configure(
                Configs.CoralSubsystem.intakeMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public void periodic() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement != null && measurement.status == LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
            System.out.println("The target is " + measurement.distance_mm + "mm away!");
        } else {
            System.out.println("Oh no! The target is out of range, or we can't get a reliable measurement!");
            // You can still use distance_mm in here, if you're ok tolerating a clamped
            // value or an unreliable measurement.
        }
    }

    private void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public Command runIntakeCommand() {
        LaserCan.Measurement measurement = lc.getMeasurement();
        if (measurement.distance_mm > 10) {
            return Commands.startEnd(
                    () -> setIntakePower(IntakeSetpoints.kForward), () -> setIntakePower(0.0));
        } else {
            return Commands.startEnd(
                    () -> setIntakePower(0.0), () -> setIntakePower(0.0));
        }
        // return Commands.startEnd(
        //         () -> setIntakePower(IntakeSetpoints.kForward), () -> setIntakePower(0.0));
    }

    public Command reverseIntakeCommand() {
        return Commands.startEnd(
                () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
    }

}
