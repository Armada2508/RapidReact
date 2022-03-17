package frc.robot.Lib.util;

import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.MathUtil;

/**
 * Contains basic functions that are used often.
 */
public class Util {
    public static final double kEpsilon = 1e-12;

    /**
     * Prevent this class from being instantiated.
     */
    private Util() {}

    /**
     * Clamps a value to a max magnitude
     * @param value The value to clamp
     * @param maxMagnitude The maximum magnitive
     * @return The clamped value
     */
    public static double clamp(double value, double maxMagnitude) {
        maxMagnitude = Math.max(0, maxMagnitude);
        return MathUtil.clamp(value, -maxMagnitude, maxMagnitude);
    }

    /**
     * Decay a value by a factor
     * @param value The value to decay
     * @param decayFactor The factor by which to decay the value
     * @return The decayed value
     */
    public static double decay(double value, double decayFactor) {
        if (value > decayFactor) {
            return value - decayFactor;
        } else if (value < -decayFactor) {
            return value + decayFactor;
        } else {
            return 0;
        }
    }

    /**
     * @param value The value to check
     * @param maxMagnitude The max absolute value of the number
     * @return If the absolute value of a number is less than a maximum
     */
    public static boolean inRange(double value, double maxMagnitude) {
        return Math.abs(value) < maxMagnitude;
    }

    /**
     * 
     * @param value The value to check
     * @param min The minimum value
     * @param max The maximum value
     * @return If the given input is within the range (min, max), both exclusive.
     */
    public static boolean inRange(double value, double min, double max) {
        return value > min && value < max;
    }

    /**
     * Applies a deadband function to the input
     * 
     * @param value The input value
     * @param threshold The deadband threshold
     * @return The deadband of the input at the set threshold
    */
    public static double deadband(double value, double threshold) {
        if(Math.abs(value) > threshold) {
            return value;
        } else {
            return 0.0;
        }
    }

    /**
     * Linearly interpolate between two values
     * @param a The first value
     * @param b The second value
     * @param x The amount to interpolate(0.0-1.0)
     * @return The interpolated value
     */
    public static double lerp(double a, double b, double x) {
        x = MathUtil.clamp(x, 0.0, 1.0);
        return a + (b - a) * x;
    }

    /**
     * Joins strings with a delimiter
     * @param delim The delimiter
     * @param strings The strings to combine
     * @return The final string
     */
    public static String joinStrings(final String delim, final List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    /**
     * Checks if the difference between two values is less than a margin of error(some small value)
     * @param a The first value
     * @param b The second value
     * @param epsilon The margin of error
     * @return If the difference between two values is less than {@code epsilon}
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }
    
    /**
     * Checks if the difference between two values is less than a margin of error(some small value)
     * @param a The first value
     * @param b The second value
     * @return If the difference between two values is less than a small {@code epsilon} value
     */
    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    /**
     * Checks if the difference between all values in a list is less than a margin of error
     * @param list The list of value
     * @param value The value to check with
     * @param epsilon The margin of error
     * @return If all of the values in the list are within {@code epsilon} of {@code value}
     */
    public static boolean allCloseTo(final List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    /**
     * Checks if the difference between all values in a list is less than a margin of error
     * @param list The list of value
     * @param value The value to check with
     * @return If all of the values in the list are within {@code epsilon} of {@code value}
     */
    public static boolean allCloseTo(final List<Double> list, double value) {
        return allCloseTo(list, value, kEpsilon);
    }

    /**
     * @param angle An angle in radians
     * @return A colinear angle from (-π-π]
     */
    public static double boundedAngle(double angle) {
        if(epsilonEquals(Math.IEEEremainder(angle, 2*Math.PI), -Math.PI)) {
            return Math.PI;
        }
        return Math.IEEEremainder(angle, 2*Math.PI);
    }

    /**
     * @param angle An angle in degrees
     * @return A colinear angle from (-180-180]
     */
    public static double boundedAngleDegrees(double angle) {
        if(epsilonEquals(Math.IEEEremainder(angle, 360), -180)) {
            return 180;
        }
        return Math.IEEEremainder(angle, 360);
    }

    /**
     * @param angle An angle in radians
     * @return A colinear angle from (0-2π]
     */
    public static double boundedAnglePositive(double angle) {
        return ((angle % (2 * Math.PI)) + 2 * Math.PI) % (2 * Math.PI);
    }

    /**
     * @param angle An angle in degrees
     * @return A colinear angle from (0-360]
     */
    public static double boundedAngleDegreesPositive(double angle) {
        return (angle % 360 + 360) % 360;
    }
    

    /**
     * @param rotation A Rotation2d object
     * @return A Rotation2d Object with an angle from (-π-π]
     */
    public static Rotation2d boundedAngle(Rotation2d rotation) {
        return new Rotation2d(boundedAngle(rotation.getRadians()));
    }
    

    /**
     * @param rotation A Rotation2d object
     * @return A Rotation2d Object with an angle from (0-2π]
     */
    public static Rotation2d boundedAnglePositive(Rotation2d rotation) {
        return new Rotation2d(boundedAnglePositive(rotation.getRadians()));
    }

    public static double inchesToMeters(double in){
        return in*.0254;
    }
}