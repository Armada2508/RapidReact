package frc.robot.Lib.vision;

import edu.wpi.first.math.MathUtil;
import frc.robot.Lib.util.Util;

public class VisionUtil {
    /**
     * Converts a pixel input to an angle output(measured in degrees)
     * 
     * @param pixel The pixel input
     * @param fov The full field of view of the camera in the desired direction
     * @param resolution The resolution of the camera in the desired direction
     * @return The angle in degrees that the input is relative to the center camera
     */
    public static double pixelsToAngles(double pixel, double fov, double resolution) {
        pixel = MathUtil.clamp(pixel, -resolution/2.0, resolution/2.0);
        return Math.toDegrees(Math.atan(Math.tan(Math.toRadians(fov/2.0))*2.0*pixel/resolution));
    }
    /**
     * Converts an angle input to a pixel output
     * 
     * @param angle The angle input(measured in degrees)
     * @param fov The full field of view of the camera in the desired direction
     * @param resolution The resolution of the camera in the desired direction
     * @return The pixel value at which the specified angle will be found
     */
    public static double anglesToPixels(double angle, double fov, double resolution) {
        angle = MathUtil.clamp(angle, -fov/2.0, fov/2.0);
        return (Math.tan(Math.toRadians(angle))*(resolution/2.0))/Math.tan(Math.toRadians(fov/2.0));
    }

    /**
     * Convert pixels to normalized coordinates(-1 to +1) from a centered pixel coordinate system(e.g. -159.5-159.5 if there were 320 pixels horizontally)
     * 
     * @param pixel The pixel value to convert
     * @param resolution The resolution of the camera in the desired direction
     * 
     * @return The normalized coordinate
     * 
     * @see {@link frc.lib.vision.VisionUtil#centerPixels(double pixel, double resolution, boolean inverted)}
     */
    public static double normalizePixels(double pixel, double resolution) {
        pixel = MathUtil.clamp(pixel, -resolution/2.0, resolution/2.0);
        return pixel / (resolution / 2.0);
    }

    /**
     * Convert pixels to centered coordinates(e.g. 159.5 to +159.5 for 320 resolution) from a pixel coordinate system(e.g. 0-319 for 320 resolution)
     * 
     * @param pixel The pixel value to convert
     * @param resolution The resolution of the camera in the desired direction
     * @param inverted If the positive and negative directions are flipped
     * @return The centered coordinate
     */
    public static double centerPixels(double pixel, double resolution, boolean inverted) {
        pixel = MathUtil.clamp(pixel, 0, resolution - 1.0);
        if(inverted) {
            return (((resolution / 2.0) - 0.5) - (double)pixel);
        } else {
            return ((double)pixel - ((resolution / 2.0) - 0.5));
        }
    }

    /**
     * Convert angles to normalized coordinates(-1 to +1) from a centered angle coordinate system(e.g -30 degrees to +30 degrees)
     * @param angle The input angle
     * @param fov The full field of view of the camera in the desired direction
     * @return The normalized angle coordinate
     */
    public static double normalizeAngle(double angle, double fov) {
        angle = MathUtil.clamp(angle, -fov/2.0, fov/2.0);
        return angle / (fov / 2.0);
    }

    /**
     * Parses corners from a double array to a CameraPoint2D array
     * @param rawCorners The corners in a double array
     * @param resolution The resolution of the camera
     * @param fov The field of view of the camera
     * @param xOffset The horizontal offset of the camera
     * @param yOffset The vertical offset of the camera
     * @return The parsed corners
     */
    public static CameraPoint2d[] parseCorners(double[] rawCorners, Resolution resolution, FOV fov, double xOffset, double yOffset) {
        CameraPoint2d[] corners = new CameraPoint2d[rawCorners.length/2];
        for (int i = 0; i < rawCorners.length; i+=2) {
            corners[i/2] = new CameraPoint2d(rawCorners[i], rawCorners[i+1], false);
            corners[i/2].center(resolution, false, true);
            corners[i/2].config(fov, resolution);
            corners[i/2].toAngle();
            corners[i/2].setY(corners[i/2].getY() + yOffset*Math.cos(Math.toRadians(corners[i/2].getX())));
            corners[i/2].setX(corners[i/2].getX() + xOffset);
        }
        return corners;
    }

    /**
     * Get the distance to the target based on its width
     * @param realTargetWidth The physical width of the target
     * @param resolution The resolution of the camera
     * @param fov The field of view of the camera
     * @param targetWidthAngle The measured width of the target(in pixels)
     * @param horizontalOffset The horizontal offset to the center of the target(in pixels)
     * @return The distance to the target in the same units as {@code targetWidth}
     */
    public static double getDistanceWidth(double realTargetWidth, Resolution resolution, FOV fov, double targetWidth, double horizontalOffset) {
  
        double x = horizontalOffset;
        double width = targetWidth;
        double angleLeft = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, fov.getX(), resolution.getX())-width/2.0, fov.getX(), resolution.getX());
        double angleRight = VisionUtil.pixelsToAngles(VisionUtil.anglesToPixels(x, fov.getX(), resolution.getX())+width/2.0, fov.getX(), resolution.getX());
        double widthAngle = angleRight-angleLeft; 
        double distance = (realTargetWidth / 2.0) / (Math.tan(Math.toRadians((widthAngle / 2.0))));
  
        return distance;
    }

    

    /**
     * Get the distance to the target based on its height
     * @param delta The difference between the height of the camera and the height of the target
     * @param targetAngle The measured angle of the target(in degrees)
     * @return The distance to the target in the same units as {@code delta}
     */
    public static double getDistanceHeight(double delta, double targetAngle) {
        return delta / Math.tan(Math.toRadians(targetAngle));
    }


      /**
       * Gets the angle of the target relative to the robot
       * @param leftCorner The left corner of the target
       * @param rightCorner The right corner of the target
       * @param realTargetWidth The physical distance from the left to the right corner
       * @param distance The distance to the target
       * @return The angle of the target relative to the robot in degrees
       */
    public static double getSkewAngle(CameraPoint2d leftCorner, CameraPoint2d rightCorner, double realTargetWidth, double distance) {
        double right = distance*Math.tan(Math.toRadians(rightCorner.getX()));
        double left = distance*Math.tan(Math.toRadians(leftCorner.getX()));
        double targetWidth = Math.abs(right - left);
        int directionMultiplier = (leftCorner.getY() > rightCorner.getY()) ? -1 : 1;
        if(targetWidth/realTargetWidth > 1.0) {
            return 0.0;
        }
        return Util.boundedAngleDegrees(Math.toDegrees(Math.acos(targetWidth/realTargetWidth))) * directionMultiplier;
    }

      /**
       * @param corners The input point array
       * @return The top two corners, the left point having index 0 and the right having index 1; returns an array of size 0 if there are not enough points.
       */
    public static CameraPoint2d[] getTopCorners(CameraPoint2d[] corners) {
        if(corners.length < 2) {
            return new CameraPoint2d[0];
        }
        CameraPoint2d[] topCorners = new CameraPoint2d[2];
        for (int i = 0; i < 2; i++) {
            topCorners[i] = corners[i];
        }
        for (int i = topCorners.length; i < corners.length; i++) {
            if(corners[i].getY() > topCorners[1].getY()) {
                topCorners[1] = corners[i];
            }
            if(topCorners[1].getY() > topCorners[0].getY()) {
                CameraPoint2d upperPoint = topCorners[1];
                topCorners[1] = topCorners[0];
                topCorners[0] = upperPoint;
            }
        }
        if(topCorners[0].getX() > topCorners[1].getX()) {
            CameraPoint2d leftPoint = topCorners[1];
            topCorners[1] = topCorners[0];
            topCorners[0] = leftPoint;
        }
        return topCorners;
    }
}