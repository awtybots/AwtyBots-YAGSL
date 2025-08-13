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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeSetpoints;
import frc.robot.Robot;

public class EndE extends SubsystemBase {
    private LaserCan lc = new LaserCan(29);
    LaserCan.Measurement measurement = lc.getMeasurement();
    // intake setup
    private SparkFlex intakeMotor = new SparkFlex(ArmConstants.IntakeCanID, MotorType.kBrushless);
    private boolean CoralEngaged = false;

    public EndE() {
        intakeMotor.configure(
                Configs.CoralSubsystem.intakeMotorConfig,
                ResetMode.kResetSafeParameters,
                PersistMode.kPersistParameters);
    }

    public boolean isCoralEngaged() {
        return lc.getMeasurement().distance_mm <= 85;}

    public void periodic() {
        //lc.getMeasurement();
        // if (measurement.distance_mm <= 85) {
        //     CoralEngaged = true;
        // } else {
        //     CoralEngaged = false;
        // }


        // if (measurement != null && measurement.status ==
        // LaserCan.LASERCAN_STATUS_VALID_MEASUREMENT) {
        // System.out.println("The target is " + measurement.distance_mm + "mm away!");
        // } else {
        // System.out.println("Oh no! The target is out of range, or we can't get a
        // reliable measurement!");
        // // You can still use distance_mm in here, if you're ok tolerating a clamped
        // // value or an unreliable measurement.
        // }
    }

    private void setIntakePower(double power) {
        intakeMotor.set(power);
    }

    public Command runIntakeCommand() {
        return Commands.run(
                () -> setIntakePower(IntakeSetpoints.kForward));
    }

    public Command NorunIntakeCommand() {
        return Commands.run(
                () -> setIntakePower(0));
    }

    public Command ArunIntakeCommandFeeder() {

        return this.runIntakeCommand().until(this::isCoralEngaged)
        .andThen(Commands.runOnce(() -> this.setIntakePower(0)));
                                        // Continuously check while running
                                         // return Commands.startEnd(
                                         // () -> setIntakePower(IntakeSetpoints.kForward), () -> setIntakePower(0.0));
    }

    
    public Command BrunIntakeCommandFeeder() {

        return Commands.startEnd(
            () -> this.setIntakePower(IntakeSetpoints.kForward), () -> this.setIntakePower(0.0)).until(this::isCoralEngaged);
                                        // Continuously check while running
                                         // return Commands.startEnd(
                                         // () -> setIntakePower(IntakeSetpoints.kForward), () -> setIntakePower(0.0));
    }

    public Command reverseIntakeCommand() {
        return Commands.startEnd(
                () -> this.setIntakePower(IntakeSetpoints.kReverse), () -> this.setIntakePower(0.0));
    }

    // public boolean isCoralEngaged() {
    //     return CoralEngaged;
    // }
}
