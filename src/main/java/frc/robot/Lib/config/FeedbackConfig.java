package frc.robot.Lib.config;

import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class FeedbackConfig {
    private final FeedbackDevice mEncoder;
    private final int mEpr;
    private final double mGearRatio;

    /**
     * Creates a new MotorConfig object to store motor controller configuration settings
     * @param encoder The type of encoder the motor uses
     * @param epr The number of edges per revolution counted by the encoder
     * @param gearRatio The gear ratio from the motor to its output
     */
    public FeedbackConfig(FeedbackDevice encoder, int epr, double gearRatio) {
        mEncoder = encoder;
        mEpr = epr;
        mGearRatio = gearRatio;
    }
    
    /**
     * @return The encoder type
     */
    public FeedbackDevice getEncoder() {
        return mEncoder;
    }

    /**
     * @return The number of edges per revolution counted by the encoder
     */
    public int getEpr() {
        return mEpr;
    }

    /**
     * @return The gear ratio from the motor to its output
     */
    public double getGearRatio() {
        return mGearRatio;
    }

    /**
     * Configure a Talon SRX with the specified FeedbackConfig
     * @param talon The Talon SRX to be configure
     * @param config The configuration to apply
     * @param slot The PID slot to use
     */
    public static void config(TalonSRX talon, FeedbackConfig config) {
        talon.configSelectedFeedbackSensor(config.getEncoder());
    }
    
    /**
     * Configure a Talon FX with the specified FeedbackConfig
     * @param talon The Talon FX to be configure
     * @param config The MotorConfig to apply
     * @param slot The PID slot to use
     */
    public static void config(TalonFX talon, FeedbackConfig config) {
        talon.configSelectedFeedbackSensor(config.getEncoder());
    }
}