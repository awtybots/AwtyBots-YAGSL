package frc.robot;

import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;

import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.subsystems.CoralSubsystem.Setpoint;

public final class Configs {
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double drivingVelocityFeedForward = 1 / ModuleConstants.kDriveWheelFreeSpeedRps;

            drivingConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(50);
            drivingConfig.encoder
                    .positionConversionFactor(drivingFactor) // meters
                    .velocityConversionFactor(drivingFactor / 60.0); // meters per second
            drivingConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(0.04, 0, 0)
                    .velocityFF(drivingVelocityFeedForward)
                    .outputRange(-1, 1);

            turningConfig
                    .idleMode(IdleMode.kBrake)
                    .smartCurrentLimit(20);
            turningConfig.absoluteEncoder
                    // Invert the turning encoder, since the output shaft rotates in the opposite
                    // direction of the steering motor in the MAXSwerve Module.
                    .inverted(true)
                    .positionConversionFactor(turningFactor) // radians
                    .velocityConversionFactor(turningFactor / 60.0); // radians per second
            turningConfig.closedLoop
                    .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                    // These are example gains you may need to them for your own robot!
                    .pid(1, 0, 0)
                    .outputRange(-1, 1)
                    // Enable PID wrap around for the turning motor. This will allow the PID
                    // controller to go through 0 to get to the setpoint i.e. going from 350 degrees
                    // to 10 degrees will go through 0 rather than the other direction which is a
                    // longer route.
                    .positionWrappingEnabled(true)
                    .positionWrappingInputRange(0, turningFactor);
        }
    }

    public static final class CoralSubsystem {
        public static final SparkFlexConfig l_elevatorMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig r_elevatorMotorConfig = new SparkFlexConfig(); 
        public static final SparkMaxConfig r_armMotorConfig = new SparkMaxConfig();
        public static final SparkMaxConfig l_armMotorConfig = new SparkMaxConfig();
        public static final SparkFlexConfig wristMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig intakeMotorConfig = new SparkFlexConfig();

        static{

                r_armMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                l_armMotorConfig.idleMode(IdleMode.kBrake).smartCurrentLimit(40).voltageCompensation(12);
                r_armMotorConfig.inverted(true);


                r_armMotorConfig.closedLoop.
                feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .outputRange(-1, 0.5)
                .maxMotion
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(.25); 

                l_armMotorConfig.closedLoop.
                feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .outputRange(-1, 0.5)
                .maxMotion
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(.25); 

                
                l_elevatorMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);
                r_elevatorMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50).voltageCompensation(12);  
                r_elevatorMotorConfig.inverted(true);

                l_elevatorMotorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.2)
                .d(.0)
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(4200)
                .maxAcceleration(6000)
                .allowedClosedLoopError(.5);

                r_elevatorMotorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.2)
                .d(.0)
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(4200)
                .maxAcceleration(6000)
                .allowedClosedLoopError(.5);

                wristMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);

                wristMotorConfig
                .closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(.25);

                intakeMotorConfig.inverted(true).idleMode(IdleMode.kBrake).smartCurrentLimit(40);

        }
    };

    public static final class FunnelIntakeSubsystem {
        public static final SparkFlexConfig l_funnelMotorConfig = new SparkFlexConfig();
        public static final SparkFlexConfig r_funnelMotorConfig = new SparkFlexConfig(); 
        public static final SparkMaxConfig funnelWristMotorConfig = new SparkMaxConfig();

        static{

                funnelWristMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(40).voltageCompensation(12);
                funnelWristMotorConfig.closedLoop.
                feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                .p(0.1)
                .outputRange(-1, 1)
                .maxMotion
                .maxVelocity(2000)
                .maxAcceleration(10000)
                .allowedClosedLoopError(.25);  
                
                l_funnelMotorConfig.idleMode(IdleMode.kCoast).smartCurrentLimit(50);
                r_funnelMotorConfig.inverted(true).idleMode(IdleMode.kCoast).smartCurrentLimit(50);

        }
    };
}