// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Configs;
import frc.robot.Constants.ClimbConstants;
import frc.robot.Constants.ClimbSetPoints;

public class Climber extends SubsystemBase {

  public enum Setpoint{
    Start,
    Climb;
  }

  private SparkFlex climberMotor = new SparkFlex(ClimbConstants.ClimbMotor, MotorType.kBrushless);
  private SparkClosedLoopController climberController = climberMotor.getClosedLoopController();
  private RelativeEncoder climberEncoder = climberMotor.getEncoder();
  private double climberCurrentTarget;


  /** Creates a new Climber. */
  public Climber() {
    climberMotor.configure(
      Configs.FunnelIntakeSubsystem.l_funnelMotorConfig,
      ResetMode.kResetSafeParameters,
      PersistMode.kPersistParameters);

    climberEncoder.setPosition(0);
  }

  private void moveToSetpoint(){
    climberController.setReference(climberCurrentTarget, ControlType.kMAXMotionPositionControl);
  }

  public Command setSetpointCommand(Setpoint setpoint){
    return this.runOnce(
      () -> {
      switch(setpoint){
        case Start:
          climberCurrentTarget = ClimbSetPoints.start;
          break;
        case Climb:
          climberCurrentTarget = ClimbSetPoints.climb;
          break;
      }
    });
  } 

  @Override
  public void periodic() {
    moveToSetpoint();
  }
}
