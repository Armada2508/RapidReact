package frc.robot.Lib.controller;

/**
 * Based on https://www.chiefdelphi.com/t/paper-take-back-half-shooter-wheel-speed-control/121640
 */
public class TakeBackHalfController {

    public double mKI;
    public double mSetpoint;
    public double mError;
    public double mLastControlEffort;
    public double mH0;
    public double mPeriod;

    /**
     * Creates a new TakeBackHalfController
     * @param kI The integral gain
     */
    public TakeBackHalfController(double kI) {
        this(kI, 0.0, 0.02);
    }

    /**
     * Creates a new TakeBackHalfController
     * @param kI The integral gain
     * @param setpoint The setpoint of the controller
     */
    public TakeBackHalfController(double kI, double setpoint) {
        this(kI, setpoint, 0.02);
    }

    /**
     * Creates a new TakeBackHalfController
     * @param kI The integral gain
     * @param setpoint The setpoint of the controller
     * @param period The period of the controller
     */
    public TakeBackHalfController(double kI, double setpoint, double period) {
        mKI = kI;
        mSetpoint = setpoint;
        mLastControlEffort = 0.0;
        mPeriod = period;
    }

    /**
     * Sets the setpoint of the controller
     * @param setpoint
     */
    public void setSetpoint(double setpoint) {
        mSetpoint = setpoint;
    }
    
    /**
     * Set the gain of the controller
     * @param kI The integral gain of the controller
     */
    public void setGain(double kI) {
        mKI = kI;
    }
    
    /**
     * Set the period of the controller
     * @param period The period of the controller
     */
    public void setPeriod(double period) {
        mPeriod = period;
    }


    /**
     * Reset the controller(does not reset gain)
     */
    public void reset() {
        mSetpoint = 0.0;
        mError = 0.0;
        mLastControlEffort = 0.0;
        mH0 = 0.0;
    }

    /**
     * Calculates the control effort based on the setpoint and input
     * @param state The current state of the system(e.g. RPM in a flywheel)
     */
    public double calculate(double state) {
        double error = mSetpoint - state;
        double controlEffort = mLastControlEffort + error * mKI * mPeriod;
        if(Math.signum(error) != Math.signum(mError)) {
            controlEffort = (mH0 + controlEffort) / 2.0;
            mH0 = controlEffort;
        }
        mError = error;
        mLastControlEffort = controlEffort;
        return controlEffort;
    }
}