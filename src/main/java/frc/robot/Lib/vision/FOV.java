package frc.robot.Lib.vision;

import frc.robot.Lib.util.Util;

public class FOV {
    public final double mX;
    public final double mY;

    /**
     * Create a new Field-of-View object
     * 
     * @param x The horizontal field of vew
     * @param y The vertical field of view
     */
    public FOV(double x, double y) {
        mX = x;
        mY = y;
    }

    /**
     * Get the horizontal FOV
     * @return The horizontal FOV
     */
    public double getX() {
        return mX;
    }

    /**
     * Get the vertical FOV
     * @return The vertical FOV
     */
    public double getY() {
        return mY;
    }

    /**
     * Prints the FOV Object
     */
    @Override
    public String toString() {
        return "Horizontal: " + getX() + ", Vertical:" + getY();
    }

    /**
     * Checks if another Object is equal to this FOV Object
     */
    @Override
    public boolean equals(Object point) {
        if(point == this) {
            return true;
        }
        if(point.getClass() == this.getClass()) {
            FOV p = (FOV) point;
            return Util.epsilonEquals(p.getX(), getX()) && Util.epsilonEquals(p.getY(), getY());
        } else {
            return false;
        }
        
    }
}