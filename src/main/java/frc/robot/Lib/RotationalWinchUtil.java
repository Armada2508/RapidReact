package frc.robot.Lib;
import frc.robot.Constants.Rotation;

public class RotationalWinchUtil{

    /**
     * @param x ajacent side length
     * @param y ajacent side length
     * @param theta D angle in degrees (angle opposite missing side)
     * @return d, final side length
     * for Side-Angle-Side triangle. Finds last side.
     */
    public static double findTriangleSideLegnth(double x, double y, double theta){
        theta *= (Math.PI/180);
        return Math.sqrt(Math.pow(x, 2) + Math.pow(y, 2) - (2*x*y*Math.cos(theta)));
    }

    public static double findRotationalWinchPos(double theta){
        return findTriangleSideLegnth(Rotation.x, Rotation.y, theta-Rotation.angleOffset) - Rotation.normalSideLength;
    }

}
