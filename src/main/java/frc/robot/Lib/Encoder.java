package frc.robot.Lib;

public class Encoder {
    /**
     * Converts from encoder units to real-world units
     * @param sensorPosition The current value read from the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return Distance traveled
     */
    public static double toDistance(double sensorPosition, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        return ((double)sensorPosition / (double)(encoderUnitsPerRev * gearRatio)) * Math.PI * wheelDiameter;
    }
    //d = s*pi*w/(e*g)
    //s = d*e*g/pi*w

    /**
     * Converts from real-world units to encoder units
     * @param distance The distance traveled
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return Distance in encoder units
     */
    public static double fromDistance(double distance, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        return (distance / ( Math.PI * wheelDiameter )) * (double)encoderUnitsPerRev * gearRatio;
    }



    /**
     * Converts encoder units per second to units per second
     * @param velocity The current velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return
     */

    public static double toVelocity(double velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        return toVelocity(velocity, encoderUnitsPerRev, gearRatio, wheelDiameter, 0.1);
    }
    
    /**
     * Converts encoder units per time unit specified to velocity
     * @param velocity The current velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @param time The time period over which the encoder velocity was measured(e.g. 0.1s for Talons)
     * @return
     */
    public static double toVelocity(double velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter, double time) {
        return toDistance(velocity, encoderUnitsPerRev, gearRatio, wheelDiameter) / time;
    }


    /**
     *  Converts from units per second to encoder units per second
     * @param velocity The current velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @return
     */

    public static double fromVelocity(double velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter) {
        return fromVelocity(velocity, encoderUnitsPerRev, gearRatio, wheelDiameter, 0.1);
    }
    
    /**
     * Converts from units per second specified to encoder units per time unit specified 
     * @param velocity The current velocity measured by the sensor
     * @param encoderUnitsPerRev The number of encoder units sensed per revolution of the output shaft of the gearbox
     * @param gearRatio The ratio of gearing from the output shaft of the gearbox to the wheel
     * @param wheelDiameter The diameter of the wheel, input units will dictate output units
     * @param time The time period over which the encoder velocity was measured(e.g. 0.1s for Talons)
     * @return
     */
    public static double fromVelocity(double velocity, int encoderUnitsPerRev, double gearRatio, double wheelDiameter, double time) {
        return fromDistance(velocity, encoderUnitsPerRev, gearRatio, wheelDiameter) * time;
    }


    /**
     * Converts encoder velocity to RPM
     * @param velocity The encoder velocity
     * @param encoderUnitsPerRev The number of encoder units per revolution
     * @param gearRatio The gear ratio between the output and the motor
     * @return RPM of the output
     */
    public static double toRPM(double velocity, int encoderUnitsPerRev, double gearRatio) {
        return toRPM(velocity, encoderUnitsPerRev, gearRatio, 0.1);
    }


    /**
     * Converts encoder velocity to RPM
     * @param velocity The encoder velocity
     * @param encoderUnitsPerRev The number of encoder units per revolution
     * @param gearRatio The gear ratio between the output and the motor
     * @param time The time over which the encoder velocity was measured(e.g. 0.1s for Talons)
     * @return RPM of the output
     */
    public static double toRPM(double velocity, int encoderUnitsPerRev, double gearRatio, double time) {
        return (velocity * 60.0 / (double)encoderUnitsPerRev) / (gearRatio * time);
    }

    /**
     * Converts RPM to encoder velocity
     * @param velocity The input RPM
     * @param encoderUnitsPerRev The number of encoder units per revolution
     * @param gearRatio The gear ratio between the output and the motor
     * @return Encoder velocity
     */
    public static double fromRPM(double RPM, int encoderUnitsPerRev, double gearRatio) {
        return fromRPM(RPM, encoderUnitsPerRev, gearRatio, 0.1);
    }

    /**
     * Converts RPM to encoder velocity
     * @param velocity The input RPM
     * @param encoderUnitsPerRev The number of encoder units per revolution
     * @param gearRatio The gear ratio between the output and the motor
     * @param time The time over which the encoder velocity is measured(e.g. 0.1s for Talons)
     * @return Encoder velocity
     */
    public static double fromRPM(double RPM, int encoderUnitsPerRev, double gearRatio, double time) {
        return RPM * ((double)encoderUnitsPerRev / 60.0) * time * gearRatio;
    }

}