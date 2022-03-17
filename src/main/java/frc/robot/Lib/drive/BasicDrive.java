package frc.robot.Lib.drive;

import frc.robot.Lib.motion.DifferentialDriveWheelPowers;
import frc.robot.Lib.util.Util;

/**
 * Helper class to implement a basic drive.
 */
public class BasicDrive {
    private double kTurnRatio;
    private double kTrimRatio;
    private double kMaxPower;
    private double kDeadbandThreshold;

    /**
     * Configure Drive Parameters
     * @param turnRatio How much the turn axis should contriute to the total drive movement
     * @param trimRatio How much the trim axis should contriute to the total drive movement
     * @param maxPower The maximum power for each side of the drivetrain
     * @param deadband The deadband for the axes
     */
    public void config(double turnRatio, double trimRatio, double maxPower, double deadband) {
        kTurnRatio = turnRatio;
        kTrimRatio = trimRatio;
        kMaxPower = maxPower;
        kDeadbandThreshold = deadband;
    }

    /**
     * A basic drive command
     * @param throttle The throttle power
     * @param turn The turn power
     * @return The wheel powers
     */
    public DifferentialDriveWheelPowers drive(double throttle, double turn) {

        throttle = Util.deadband(throttle, kDeadbandThreshold);
        turn = Util.deadband(turn, kDeadbandThreshold);
        
        turn *= kTurnRatio;

        double powerL = throttle + turn;
        double powerR = throttle - turn;

        powerR *= kMaxPower;
        powerL *= kMaxPower;

        double turningPower = powerL - powerR;
        if(turningPower > 0 && powerL > kMaxPower) {
            powerL = kMaxPower;
            powerR = kMaxPower - turningPower;
        } else if(turningPower < 0 && powerR > kMaxPower) {
            powerR = kMaxPower;
            powerL = kMaxPower + turningPower;
        }
        
        return new DifferentialDriveWheelPowers(powerL, powerR);

    }

    /**
     * A basic drive command
     * @param throttle The throttle power
     * @param turn The turn power
     * @param trim The trim power
     * @return The wheel powers
     */
    public DifferentialDriveWheelPowers drive(double throttle, double turn, double trim) {

        throttle = Util.deadband(throttle, kDeadbandThreshold);
        trim = Util.deadband(trim, kDeadbandThreshold);
        turn = Util.deadband(turn, kDeadbandThreshold);
        
        turn *= kTurnRatio;
        trim *= kTrimRatio;
        turn += trim; 

        double powerL = throttle + turn;
        double powerR = throttle - turn;

        powerR *= kMaxPower;
        powerL *= kMaxPower;

        double turningPower = powerL - powerR;
        if(turningPower > 0 && powerL > kMaxPower) {
            powerL = kMaxPower;
            powerR = kMaxPower - turningPower;
        } else if(turningPower < 0 && powerR > kMaxPower) {
            powerR = kMaxPower;
            powerL = kMaxPower + turningPower;
        }
        
        return new DifferentialDriveWheelPowers(powerL, powerR);

    }
}
