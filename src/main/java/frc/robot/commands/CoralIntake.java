package frc.robot.commands;

import au.grapplerobotics.LaserCan;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeSetpoints;
import frc.robot.subsystems.EndE;

public class CoralIntake extends Command {

    private EndE intake;
    private LaserCan lc = new LaserCan(29);
    
    private double forward = IntakeSetpoints.kReverse;


    public CoralIntake(EndE intake) {
        this.intake = intake;
        // Use addRequirements() here to declare subsystem dependencies.
        // addRequirements(Robot.coral);
    }

    @Override
    public void initialize() {
        intake.setIntakePower(0.0); // Ensure intake is stopped at the start
    }

    @Override
    public void execute(){
        LaserCan.Measurement measurement = lc.getMeasurement();
        if(measurement != null && measurement.distance_mm <= 85) {
            //withTimeout(0.2);
            intake.setIntakePower(0.0);
        } else {
            intake.setIntakePower(forward);
        }
    }

    @Override
    public boolean isFinished() {
        return true; // This command finishes immediately after starting
    }
    
}
