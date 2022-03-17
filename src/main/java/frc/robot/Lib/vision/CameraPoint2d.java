package frc.robot.Lib.vision;

import frc.robot.Lib.util.Util;

public class CameraPoint2d {
    private double mX;
    private double mY;
    private boolean mIsAngle;
    private FOV mFov;
    private Resolution mRes;

    /**
     * Creates a new CameraPoint2d
     * 
     * @param tx The x angle of the target relative to the camera
     * @param ty The y angle of the target relative to the camera
     */
    public CameraPoint2d(double tx, double ty) {
        this(tx, ty, true);
    }
    
    /**
     * Creates a new CameraPoint2d
     * 
     * @param x The x coordinate
     * @param y The y coordinate
     * @param angle If the coordinate values are in degrees
     */
    public CameraPoint2d(double x, double y, boolean angle) {
        mIsAngle = angle;
        mX = x;
        mY = y;
    }

    /**
     * Get the x coordinate
     * @return The x coordinate
     */
    public double getX() {
        return mX;
    }

    /**
     * Set the y coordinate
     * @param y The value to set the y coordinate to
     */
    public void setY(double y) {
        mY = y;
    }

    /**
     * Set the x coordinate
     * @param x The value to set the x coordinate to
     */
    public void setX(double x) {
        mX = x;
    }

    /**
     * Sets this point's coordinates to the values given
     * @param x The new x coordinate
     * @param y The new y coordinate
     */
    public void set(double x, double y) {
        mX = x;
        mY = y;
    }

    /**
     * Get the y coordinate
     * @return The y coordinate
     */

    public double getY() {
        return mY;
    }

    /**
     * Check if the CameraPoint2d is an angle
     * @return If the CameraPoint2d is an angle
     */

    public boolean isAngle() {
        return mIsAngle;
    }

    /**
     * Center the CameraPoint2d
     * 
     * @param resolution The resolution of the camera
     * @param xInverted If the x coordinate is inverted
     * @param yInverted If the y coordinate is inverted
     */
    public void center(Resolution resolution, boolean xInverted, boolean yInverted) {
        if(!mIsAngle) {
            mX = VisionUtil.centerPixels(mX, (double)resolution.getX(), xInverted);
            mY = VisionUtil.centerPixels(mY, (double)resolution.getY(), yInverted);
        }
    }


    /**
     * Converts the CameraPoint2d to an angle
     * @param fov The field-of-view of the camera
     * @param resolution The resolution of the camera
     */
    public void toAngle(FOV fov, Resolution resolution) {
        if(!mIsAngle) {
            mX = VisionUtil.pixelsToAngles(mX, fov.getX(), resolution.getX());
            mY = VisionUtil.pixelsToAngles(mY, fov.getY(), resolution.getY());
            mIsAngle = true;
        }
    }

    
    /**
     * Converts the CameraPoint2d to pixel coordinates
     * @param fov The field-of-view of the camera
     * @param resolution The resolution of the camera
     */
    public void toPixels(FOV fov, Resolution resolution) {
        if(mIsAngle) {
            mX = VisionUtil.anglesToPixels(mX, fov.getX(), resolution.getX());
            mY = VisionUtil.anglesToPixels(mY, fov.getY(), resolution.getY());
            mIsAngle = false;
        }
    }


    /**
     * Stores camera parameters for point
     * @param fov
     * @param resolution
     */
    public void config(FOV fov, Resolution resolution) {
        mFov = fov;
        mRes = resolution;
    }

    
    /**
     * Converts the CameraPoint2d to an angle using the FOV and Resolution set globally with {@link frc.lib.vision.CameraPoint2d#config(FOV, Resolution)}
     */
    public void toAngle() {
        if(mFov == null || mRes == null) {
            return;
        }
        if(!mIsAngle) {
            mX = VisionUtil.pixelsToAngles(mX, mFov.getX(), mRes.getX());
            mY = VisionUtil.pixelsToAngles(mY, mFov.getY(), mRes.getY());
            mIsAngle = true;
        }
    }

    
    /**
     * Converts the CameraPoint2d to pixel coordinates using the FOV and Resolution set globally with {@link frc.lib.vision.CameraPoint2d#config(FOV, Resolution)}
     */
    public void toPixels() {
        if(mFov == null || mRes == null) {
            return;
        }
        if(mIsAngle) {
            mX = VisionUtil.anglesToPixels(mX, mFov.getX(), mRes.getX());
            mY = VisionUtil.anglesToPixels(mY, mFov.getY(), mRes.getY());
            mIsAngle = false;
        }
    }

    /**
     * Prints the CameraPoint2d Object
     */
    @Override
    public String toString() {
        return "X: " + getX() + ", Y:" + getY();
    }

    /**
     * Checks if another Object is equal to this CameraPoint2d
     */
    @Override
    public boolean equals(Object point) {
        if(point == this) {
            return true;
        }
        if(point.getClass() == this.getClass()) {
            CameraPoint2d p = (CameraPoint2d) point;
            return Util.epsilonEquals(p.getX(), getX()) && Util.epsilonEquals(p.getY(), getY()) && (p.isAngle() == isAngle());
        } else {
            return false;
        }
        
    }

    public static CameraPoint2d midpoint(CameraPoint2d point1, CameraPoint2d point2) {
        if(point1.isAngle() != point2.isAngle()) return new CameraPoint2d(0.0, 0.0);
        return new CameraPoint2d((point1.getX()+point2.getX())/2.0, (point1.getY()+point2.getY())/2.0, point1.isAngle());
    }

}