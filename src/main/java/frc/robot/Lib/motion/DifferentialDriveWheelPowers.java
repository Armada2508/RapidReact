package frc.robot.Lib.motion;

import frc.robot.Lib.util.Util;

public class DifferentialDriveWheelPowers {
    private double mRPower;
    private double mLPower;

    /**
     * An object that stores two wheel powers
     * @param lPower The left wheel power
     * @param rPower The right wheel power
     */
    public DifferentialDriveWheelPowers(double lPower, double rPower) {
        mRPower = rPower;
        mLPower = lPower;
    }

    /**
     * An object that stores two wheel positions.<br>
     * <br>
     * This object will be initialized with both wheel positions at 0.0
     */
    public DifferentialDriveWheelPowers() {
        this(0.0, 0.0);
    }

    /**
     * @return The right wheel power
     */

    public double getRight() {
        return mRPower;
    }

    /**
     * @return The left wheel power
     */

    public double getLeft() {
        return mLPower;
    }

    /**
     * Deadband the powers
     * @param threshold The threshold
     */
    public void deadband(double threshold) {
        mRPower = Util.deadband(mRPower, threshold);
        mLPower = Util.deadband(mLPower, threshold);
    }

    /**
     * Prints both wheel positions
     */
    @Override
    public String toString() {
        return ("Left Wheel Power: " + mLPower + ", Right Wheel Power: " + mRPower);
    }
}