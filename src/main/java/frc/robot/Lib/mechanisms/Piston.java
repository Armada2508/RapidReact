package frc.robot.Lib.mechanisms;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

public class Piston {

    private Solenoid mRetract;
    private Solenoid mExtend;
    private boolean mExtendInverted;
    private boolean mRetractInverted;

    /**
     * Creates a new Piston Object
     * @param retractSolenoid The ID of the solenoid connected to the piston that causes it to retract
     * @param extendSolenoid The ID of the solenoid connected to the piston that causes it to extend
     */
    public Piston(int retractSolenoid, int extendSolenoid) {
        mRetract = new Solenoid(PneumaticsModuleType.CTREPCM, retractSolenoid);
        mExtend = new Solenoid(PneumaticsModuleType.CTREPCM, extendSolenoid);
    }

    /**
     * Sets if the extend solenoid is inverted
     * @param inverted If the extend solenoid is inverted
     */
    public void extendInverted(boolean inverted) {
        mExtendInverted = inverted;
    }


    /**
     * Sets if the retract solenoid is inverted
     * @param inverted If the retract solenoid is inverted
     */
    public void retractInverted(boolean inverted) {
        mRetractInverted = inverted;
    }

    /**
     * Extend the Piston
     */
    public void extend() {
        mRetract.set(mRetractInverted);
        mExtend.set(!mExtendInverted);
    }

    /**
     * Retract the Piston
     */
    public void retract() {
        mRetract.set(!mRetractInverted);
        mExtend.set(mExtendInverted);
    }

    /**
     * Vent the Piston
     */
    public void vent() {
        mRetract.set(mRetractInverted);
        mExtend.set(mExtendInverted);
    }

    /**
     * Turn off all solenoids
     */
    public void disable() {
        mRetract.set(false);
        mExtend.set(false);
    }
}