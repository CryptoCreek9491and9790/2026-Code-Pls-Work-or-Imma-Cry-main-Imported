package frc.robot;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.UtilityConstants;

public final class Configs {


        
    public static final class MAXSwerveModule {
        public static final SparkMaxConfig drivingConfig = new SparkMaxConfig();
        public static final SparkMaxConfig turningConfig = new SparkMaxConfig();

        static {
            // Use module constants to calculate conversion factors and feed forward gain.
            double drivingFactor = ModuleConstants.kWheelDiameterMeters * Math.PI
                    / ModuleConstants.kDrivingMotorReduction;
            double turningFactor = 2 * Math.PI;
            double nominalVoltage = 12.0;
            double drivingVelocityFeedForward = nominalVoltage / ModuleConstants.kDriveWheelFreeSpeedRps;

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
                    .outputRange(-1, 1)
                    .feedForward.kV(drivingVelocityFeedForward);
        
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


    public static final class IntakeSubsystem {
        public static final SparkMaxConfig INTAKE_CONFIG = new SparkMaxConfig();
        public static final SparkMaxConfig PIVOT_CONFIG = new SparkMaxConfig();

        static{
                INTAKE_CONFIG
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .openLoopRampRate(.5)
                .smartCurrentLimit(40);

                double pivotDegreesPerRotation = 360;

                PIVOT_CONFIG
                .inverted(false)
                .idleMode(IdleMode.kBrake)
                .smartCurrentLimit(40);
                PIVOT_CONFIG.absoluteEncoder
                .positionConversionFactor(pivotDegreesPerRotation)
                .velocityConversionFactor(pivotDegreesPerRotation / 60);
                PIVOT_CONFIG.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
                //Make sure to tune PID gains- start with a small P value liek .01
                // and increase until it reaches the setpoint without oscillation 
                .pid(.01, 0, 0)
                .outputRange(-.3, .3) //Limits max power for safety while tuning
                .positionWrappingEnabled(true);
        }
        }
        public static final class ShooterSubsystem {
                public static final SparkMaxConfig SHOOTER_CONFIG = new SparkMaxConfig();
                public static final SparkMaxConfig BACKROLLER_CONFIG = new SparkMaxConfig();

                static {


                        SHOOTER_CONFIG
                        .inverted(false)
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40);
                        SHOOTER_CONFIG.encoder
                        .velocityConversionFactor(1); //Native RPM
                        SHOOTER_CONFIG.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(.00005, 0, 0)
                        .outputRange(-1, 1)
                        .feedForward.kV(12.0 / UtilityConstants.kVortexFreeSpeedRpm);


                        BACKROLLER_CONFIG
                        .inverted(false)
                        .idleMode(IdleMode.kCoast)
                        .smartCurrentLimit(40);
                        BACKROLLER_CONFIG.encoder
                        .velocityConversionFactor(1); //Native RPM
                        BACKROLLER_CONFIG.closedLoop
                        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
                        .pid(.00005, 0, 0)
                        .outputRange(-1, 1)
                        .feedForward.kV(1 / UtilityConstants.kVortexFreeSpeedRps / 60);
                  }
                }
        public static final class HopperSubsystem {
                public static final SparkMaxConfig ROLLER_CONFIG = new SparkMaxConfig();
                public static final SparkMaxConfig INDEXER_CONFIG = new SparkMaxConfig();

        static {
                ROLLER_CONFIG
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .openLoopRampRate(1)
                .smartCurrentLimit(40);

                INDEXER_CONFIG
                .inverted(false)
                .idleMode(IdleMode.kCoast)
                .openLoopRampRate(1)
                .smartCurrentLimit(40);
        }
        }

        }
