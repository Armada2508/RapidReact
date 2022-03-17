package frc.robot.Lib.drive;

import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.MathUtil;
import frc.robot.Lib.util.Util;

/**
 * Helper class to implement curvature controlled drive. This helps make the robot more controllable at high
 * speeds.
 */
public class SmoothDrive {

    private static double kThrottleDeadband = 0.035;
    private static double kTurnDeadband = 0.02;

    private static double kTurnNonLinearity = 0.5;

    private static double kInertialCompensationThreshold = 0.65;
    private static double kTurningInertialCompensation = 3.0;
    private static double kSmallInertialCompensation = 3.0;
    private static double kLargeInertialCompensation = 4.0;

    private static double kTurnSensitivity = 0.625;

    private static double kQuickTurnScalar = .65;

    private static double kQuickStopDeadband = 0.5;
    private static double kQuickStopFactor = 0.125;
    private static double kMaxQuickStopAccumulator = 2.8;

    private static double kIntertialCompensationAccumulatorDecayFactor = 1.0;

    private double mLastTurn = 0.0;
    private double mQuickStopAccumulator = 0.0;
    private double mIntertialCompensationAccumulator = 0.0;

    public DifferentialDriveWheelSpeeds smoothDrive(double throttle, double turn, boolean isQuickTurn, double maxVelocity) {
        // Deadband the throttle and turn
        turn =  Util.deadband(turn, kTurnDeadband);
        throttle = Util.deadband(throttle, kThrottleDeadband);

        // Find the negative change in turn
        double inertialCompensation = turn - mLastTurn;
        mLastTurn = turn;

        // Apply a sinusoid to make turning feel better
        turn = applySinusoid(turn, kTurnNonLinearity, 3);

        // Compensate for Inertia
        double intertialCompensationFactor;
        if (turn * inertialCompensation > 0) {
            // If we are going into a turn, set the Inertial Compensation Factor accordingly
            intertialCompensationFactor = kTurningInertialCompensation;
        } else {
            if (Math.abs(turn) > kInertialCompensationThreshold) {
                // If we are not going into a turn and are above the threshold, set the Inertial Compensation Factor accordingly
                intertialCompensationFactor = kLargeInertialCompensation;
            } else {
                // If we are not going into a turn and are not above the threshold, set the Inertial Compensation Factor accordingly
                intertialCompensationFactor = kSmallInertialCompensation;
            }
        }
        
        // Calculate the Final Inertial Compensation Power
        double intertialCompensationPower = inertialCompensation * intertialCompensationFactor;
        mIntertialCompensationAccumulator += intertialCompensationPower;

        // Augment turn with Inertial Compensation
        turn = turn + mIntertialCompensationAccumulator;

        // Decay Intertial Compensation Accumulator
        mIntertialCompensationAccumulator = Util.decay(mIntertialCompensationAccumulator, kIntertialCompensationAccumulatorDecayFactor);

        double keepDelta, angularPower;

        // If quick turning
        if (isQuickTurn) {
            if (Math.abs(throttle) < kQuickStopDeadband) {
                // If less than the quick stop deadband, calculate the quick stop accumulator
                mQuickStopAccumulator = (1 - kQuickStopFactor) * mQuickStopAccumulator + kQuickStopFactor * MathUtil.clamp(turn, -1.0, 1.0) * kMaxQuickStopAccumulator;
            }
            // Keep the difference between the speeds
            keepDelta = 1.0;
            // Calculate the angular power
            angularPower = turn * kQuickTurnScalar;
        } else {
            // Don't keep the difference between the speeds
            keepDelta = 0.0;
            // Calculate the angular power(proportional to speed)
            angularPower = Math.abs(throttle) * turn * kTurnSensitivity - mQuickStopAccumulator;
            mQuickStopAccumulator = Util.decay(mQuickStopAccumulator, 1.0);
        }

        // Calculate Powers
        double rightPower, leftPower;

        rightPower = leftPower = throttle;
        leftPower += angularPower;
        rightPower -= angularPower;

        if (leftPower > 1.0) {
            rightPower -= keepDelta * (leftPower - 1.0);
            leftPower = 1.0;
        } else if (rightPower > 1.0) {
            leftPower -= keepDelta * (rightPower - 1.0);
            rightPower = 1.0;
        } else if (leftPower < -1.0) {
            rightPower += keepDelta * (-1.0 - leftPower);
            leftPower = -1.0;
        } else if (rightPower < -1.0) {
            leftPower += keepDelta * (-1.0 - rightPower);
            rightPower = -1.0;
        }
        return new DifferentialDriveWheelSpeeds(leftPower * maxVelocity, rightPower * maxVelocity);
    }


    

    /**
     * Applies a sinusoid to the input
     * @param input The input value
     * @param factor How close to sinusoidial the output should be, (0.0-1.0]
     * @param recursionNumber How many times to apply the function
     * @return The scaled value
     */
    private static double applySinusoid(double input, double factor, int recursionNumber) {
        recursionNumber--;
        if(recursionNumber > 0) {
            return applySinusoid(input, factor, recursionNumber);
        }
        factor = MathUtil.clamp(factor, Util.kEpsilon, 1.0);
        double numerator = Math.sin(Math.PI / 2.0 * factor * input);
        double denominator = Math.sin(Math.PI / 2.0 * factor);
        return numerator / denominator;
    }
}
