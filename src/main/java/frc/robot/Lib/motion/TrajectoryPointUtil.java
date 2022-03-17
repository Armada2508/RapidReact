package frc.robot.Lib.motion;

import com.ctre.phoenix.motion.*;

public class TrajectoryPointUtil {
    /**
     * 
     * @param point The point to reset
     */
    public static void resetPoint(TrajectoryPoint point) {
        point.auxiliaryArbFeedFwd = 0;
        point.auxiliaryVel = 0;
        point.auxiliaryPos = 0;
        point.arbFeedFwd = 0;
        point.velocity = 0;
        point.position = 0;
        point.headingDeg = 0;
        point.profileSlotSelect0 = 0;
        point.profileSlotSelect1 = 0;
        point.timeDur = 0;
        point.useAuxPID = false;
        point.zeroPos = false;
        point.isLastPoint = false;
        point.zeroPos = false;
    }
}