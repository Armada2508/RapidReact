package frc.robot.Lib.vision;

import frc.robot.Lib.util.Util;

public class Resolution {
    private final int mX;
    private final int mY;

    /**
     * Create a new Resolution object
     * 
     * @param x The horizontal resolution
     * @param y The vertical resolution
     */
    public Resolution(int x, int y) {
        mX = x;
        mY = y;
    }

    /**
     * Get the horizontal resolution
     * @return The horizontal resolution
     */
    public int getX() {
        return mX;
    }

    /**
     * Get the vertical resolution
     * @return The vertical resolution
     */
    public int getY() {
        return mY;
    }

    /**
     * Prints the Resolution Object
     */
    @Override
    public String toString() {
        return "Horizontal: " + getX() + ", Vertical:" + getY();
    }

    /**
     * Checks if another Object is equal to this Resolution Object
     */
    @Override
    public boolean equals(Object point) {
        if(point == this) {
            return true;
        }
        if(point.getClass() == this.getClass()) {
            Resolution p = (Resolution) point;
            return Util.epsilonEquals(p.getX(), getX()) && Util.epsilonEquals(p.getY(), getY());
        } else {
            return false;
        }
        
    }
}