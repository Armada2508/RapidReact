package frc.robot.Lib.config;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

public class FeedbackConstants {

    private final double mP;
    private final double mI;
    private final double mD;
    private final double mF;
    private final double mMia;
    
    /**
     * Creates a new FeedbackConstants object
     * @param p The proportional constant
     * @param i The integral constant
     * @param d The derivative constant
     * @param f The feed-forward constant
     * @param mia The maximum integral accumulator constant
     */
    public FeedbackConstants(double p, double i, double d, double f, int mia) {
        mP = p;
        mI = i;
        mD = d;
        mF = f;
        mMia = mia;
    }

    /**
     * @return The proportional constant
     */
    public double getP() {
        return mP;
    }

    /**
     * @return The integral constant
     */
    public double getI() {
        return mI;
    }

    /**
     * @return The derivative constant
     */
    public double getD() {
        return mD;
    }

    /**
     * @return The feed-forward constant
     */
    public double getF() {
        return mF;
    }

    /**
     * @return The max integral accumulator constant
     */
    public double getMIA() {
        return mMia;
    }
    
    /**
     * Configure a Talon SRX with the specified FeedbackConstants
     * @param talon The Talon SRX to be configure
     * @param config The configuration to apply
     * @param slot The PID slot to use
     */
    public static void config(TalonSRX talon, FeedbackConstants config, int slot) {
        talon.config_kP(slot, config.getP());
        talon.config_kI(slot, config.getI());
        talon.config_kD(slot, config.getD());
        talon.config_kF(slot, config.getF());
        talon.configMaxIntegralAccumulator(slot, config.getMIA());
    }
    
    /**
     * Configure a Talon FX with the specified FeedbackConstants
     * @param talon The Talon FX to be configure
     * @param config The MotorConfig to apply
     * @param slot The PID slot to use
     */
    public static void config(TalonFX talon, FeedbackConstants config, int slot) {
        talon.config_kP(slot, config.getP());
        talon.config_kI(slot, config.getI());
        talon.config_kD(slot, config.getD());
        talon.config_kF(slot, config.getF());
        talon.configMaxIntegralAccumulator(slot, config.getMIA());
    }

}