package frc.robot;

import frc.robot.Lib.RotationalWinchUtil;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Lib.util.Util;
import frc.robot.Lib.config.FeedbackConfig;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import frc.robot.Lib.vision.Resolution;

public class Constants {
    
    public static final class Drive {
        public static final int LFID = 0;
        public static final int LRID = 1;
        public static final int RFID = 2;
        public static final int RRID = 3;
        public static final int turnAdjustment = 3; 

        public static final double diameter = 6;
        public static final int encoderUnits = 2048;
        public static final double gearboxRatio = 12.75;
        public static final double kTrackWidth = Util.inchesToMeters(23.5);

        public static final FeedbackConfig kFeedbackConfig = new FeedbackConfig(FeedbackDevice.IntegratedSensor, encoderUnits, gearboxRatio);
    }

    public static final class Winch {
        public static final double diameter = 1.5 + .25; //1.75 comes from 1.5 diameter of spool + .25 diameter of cord
        public static final int encoderUnits = 2048;
    }    

    public static final class Linear {
        public static final int leftID = 4; 
        public static final int rightID = 5; 
        public static final double power = .3;
        public static final double max = 27;
        public static final double min = 0;                                 
        public static final PIDController linearController = new PIDController(.0002, .00001, .000020);
        public static final int gearboxRatio = 20;

    }

    public static final class Rotation {
        public static final int leftID = 6;
        public static final int rightID = 7; 
        public static final double power = .25;


        public static final double x = 8.5;
        public static final double y = 9.5;
        public static final double angleOffset = 30;
        public static final double normalSideLength = RotationalWinchUtil.findTriangleSideLegnth(x, y, 90-angleOffset);
        public static final PIDController rotationController = new PIDController/*(0, 0, 0)*/(.002, .001, .002);
        public static final int gearboxRatio = 36;

        public static final double max = RotationalWinchUtil.findRotationalWinchPos(135);
        public static final double min = RotationalWinchUtil.findRotationalWinchPos(70);

    }

    // ========================================
    //    Global Motor Controller Constants
    // ========================================
    public static final class MotorController {

        // Status frames are sent over CAN that contain data about the Talon.
        // They are broken up into different pieces of data and the frequency
        // at which they are sent can be changed according to your needs.
        // The period at which their are sent is measured in ms

        public static final int kTalonFrame1Period = 20;  // How often the Talon reports basic info(Limits, limit overrides, faults, control mode, invert)
        public static final int kTalonFrame2Period = 20;  // How often the Talon reports sensor info(Sensor position/velocity, current, sticky faults, profile)
        public static final int kTalonFrame3Period = 160;  // How often the Talon reports non selected quad info(Position/velocity, edges, quad a and b pin, index pin)
        public static final int kTalonFrame4Period = 160;  // How often the Talon reports additional info(Analog position/velocity, temperature, battery voltage, selected feedback sensor)
        public static final int kTalonFrame8Period = 160;  // How often the Talon reports more encoder info(Talon Idx pin, PulseWidthEncoded sensor velocity/position)
        public static final int kTalonFrame10Period = 160;  // How often the Talon reports info on motion magic(Target position, velocity, active trajectory point)
        public static final int kTalonFrame13Period = 160; // How often the Talon reports info on PID(Error, Integral, Derivative)

    }
    
    

    public static final class Camera {
         
        // Camera Constants
        public static final Resolution kCameraResolution = new Resolution(160, 120); // The resolution to use for streaming the camera to the dashboard
        public static final int kCameraFPS = 14;  // The FPS to use when streaming the camera to the dashboard
        public static final int kCameraCompression = 90; // The compression to use when streaming the camera to the dashboard
    }   

}
