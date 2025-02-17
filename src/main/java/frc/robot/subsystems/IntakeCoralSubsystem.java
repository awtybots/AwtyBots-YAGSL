package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkFlexConfig;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.IntakeSetpoints;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class IntakeCoralSubsystem extends SubsystemBase {
    
    private final SparkFlex intakeMotor;
    private final SparkFlexConfig intakeMotorConfig;

    public IntakeCoralSubsystem() {
        intakeMotor = new SparkFlex(ArmConstants.IntakeCanID, MotorType.kBrushless);
        intakeMotorConfig = new SparkFlexConfig();

        intakeMotorConfig.inverted(false).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        intakeMotor.configure(intakeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    }

    @Override
    public void periodic() {
      
    }
  

     public void intake() {
      intakeMotor.set(1); //test this
    }

    public void outtake() {
      intakeMotor.set(IntakeSetpoints.kReverse); //test this
    }

    public void intakeStop() {
      intakeMotor.set(IntakeSetpoints.kHold); //test this);
    }
  
    public double getCurrent() {
      return intakeMotor.getOutputCurrent();
    }
}
