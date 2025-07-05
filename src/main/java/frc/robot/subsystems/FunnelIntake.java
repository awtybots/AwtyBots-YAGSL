package frc.robot.subsystems;

//import com.revrobotics.RelativeEncoder;

//import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
//import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.FunnelConstants;
import frc.robot.Constants.FunnelIntakeSetpoints;
//import frc.robot.Constants.FunnelWristSetpoints;

public class FunnelIntake extends SubsystemBase {
  public enum Setpoint{
    FeederStation,
    Climb;
}
 // funnel setup
  private SparkFlex l_funnelMotor = new SparkFlex(FunnelConstants.FunnelLIntake, MotorType.kBrushless);
  
  
  public FunnelIntake() {
    l_funnelMotor.configure(
      Configs.FunnelIntakeSubsystem.l_funnelMotorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    
  }

  

  
  public void setIntakePower(double leftPower, double rightPower) {
    l_funnelMotor.set(leftPower);
  
  }

  public Command runIntakeCommand(){
        return this.startEnd(
            () -> this.setIntakePower(FunnelIntakeSetpoints.kForward, FunnelIntakeSetpoints.kReverse), () -> this.setIntakePower(0.0, 0.0));
  }
  

 
  


    
}