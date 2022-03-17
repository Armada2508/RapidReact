package frc.robot.Lib.config;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class MotorConfig {
    private final int mMaxContCurrent;
    private final int mMaxPeakCurrent;
    private final int mPeakCurrentDuration;
    private final double mDeadband;

    /**
     * Creates a new MotorConfig object to store motor controller configuration settings
     * @param maxCurrent The maximum current the motor will draw continuously
     * @param maxPeakCurrent The maximum current the motor will draw at peak
     * @param peakCurrentDuration The duration of the peak current(in ms)
     * @param deadband The deadband(0.0-1.0) to be applied to the motor controller
     */
    public MotorConfig(int maxContCurrent, int maxPeakCurrent, int peakCurrentDuration, double deadband) {
        mMaxContCurrent = maxContCurrent;
        mMaxPeakCurrent = maxPeakCurrent;
        mPeakCurrentDuration = peakCurrentDuration;
        mDeadband = deadband;
    }
    
    /**
     * @return The max current the motor will draw continuously
     */
    public int getContinuousCurrent() {
        return mMaxContCurrent;
    }

    /**
     * @return The max current the motor will draw at peak
     */
    public int getPeakCurrent() {
        return mMaxPeakCurrent;
    }

    /**
     * @return The maximum current the motor will draw at peak
     */
    public int getPeakDuration() {
        return mPeakCurrentDuration;
    }

    /**
     * @return The size of the deadband
     */
    public double getDeadband() {
        return mDeadband;
    }
    
    
    /**
     * Configure a Talon SRX with the specified MotorConfig
     * @param talon The Talon SRX to be configure
     * @param config The configuration to apply
     */
    public static void config(TalonSRX talon, MotorConfig config) {
        talon.configContinuousCurrentLimit(config.getContinuousCurrent());
        talon.configPeakCurrentLimit(config.getPeakCurrent());
        talon.configPeakCurrentDuration(config.getPeakDuration());
        talon.enableCurrentLimit(true);
        talon.configNeutralDeadband(config.getDeadband());
    }
    
    /**
     * Configure a Talon FX with the specified MotorConfig
     * @param talon The Talon FX to be configure
     * @param config The MotorConfig to apply
     */
    public static void config(TalonFX talon, MotorConfig config) {
        talon.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, config.getContinuousCurrent(), config.getPeakCurrent(), config.getPeakDuration()));
        talon.configNeutralDeadband(config.getDeadband());
    }

    /**
     * Resets a Talon SRX's constants
     * @param talon The Talon SRX to be reset
     */
    public static void resetTalon(TalonSRX talon) {
        talon.configFactoryDefault();
    }

    /**
     * Resets a Talon FX's constants
     * @param talon The Talon FX to be reset
     */
    public static void resetTalon(TalonFX talon) {
        talon.configFactoryDefault();
    }
}