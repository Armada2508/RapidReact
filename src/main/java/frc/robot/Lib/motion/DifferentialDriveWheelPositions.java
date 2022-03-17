package frc.robot.Lib.motion;

public class DifferentialDriveWheelPositions {
    private double mRWheelPos;
    private double mLWheelPos;

    /**
     * An object that stores two wheel positions
     * @param lWheelPos The left wheel position
     * @param rWheelPos The right wheel position
     */
    public DifferentialDriveWheelPositions(double lWheelPos, double rWheelPos) {
        mLWheelPos = lWheelPos;
        mRWheelPos = rWheelPos;
    }

    /**
     * An object that stores two wheel positions.<br>
     * <br>
     * This object will be initialized with both wheel positions at 0.0
     */
    public DifferentialDriveWheelPositions() {
        this(0.0, 0.0);
    }

    /**
     * @return The right wheel position
     */

    public double getRight() {
        return mRWheelPos;
    }

    /**
     * @return The left wheel position
     */

    public double getLeft() {
        return mLWheelPos;
    }

    /**
     * Prints both wheel positions
     */
    @Override
    public String toString() {
        return ("Left Wheel Position: " + mLWheelPos + ", Right Wheel Position: " + mRWheelPos);
    }
}