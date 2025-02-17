package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeCoralSubsystem;

public class AutoIntake extends Command{

    IntakeCoralSubsystem intake;
    int counter = 0;

    public AutoIntake(IntakeCoralSubsystem intake){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void execute(){
        this.intake.intake();
        
        if(intake.getCurrent() >= 25){
            counter++;
        }else{
                counter = 0;
            }
        }        

    @Override
    public boolean isFinished() {
        if(counter > 10){
          return true;
        }else{
          return false;
        }
      }
}