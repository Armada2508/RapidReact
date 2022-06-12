package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.Command;
//import frc.robot.commands.HangWait;
import frc.robot.commands.MotionMagicAutoClimb;
import frc.robot.commands.MotionMagicCommand;
import frc.robot.commands.MotionMagicCommandHang;
import frc.robot.commands.AutoDrive;
import frc.robot.commands.Callibrate;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.OneMotorCommand;
import frc.robot.commands.SetYawCommand;
import frc.robot.commands.WinchCommand;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.Constants.Linear;
import frc.robot.Constants.Rotation;
import frc.robot.Constants.Intake;
import frc.robot.Lib.RotationalWinchUtil;
import frc.robot.commands.AutoCalibrate;
//import frc.robot.commands.AutoClimb;
import frc.robot.commands.AutoTurn;
import edu.wpi.first.cscore.CameraServerJNI;
import edu.wpi.first.cscore.CvSink;
import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.cscore.UsbCamera;
//import edu.wpi.first.cscore.VideoSink;
//import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import edu.wpi.first.cscore.VideoSource.ConnectionStrategy;
import edu.wpi.first.cameraserver.CameraServer;
//import edu.wpi.first.cameraserver.CameraServerShared;
//import edu.wpi.first.cscore.CameraServerCvJNI;
//import edu.wpi.first.cscore.CameraServerJNI;
//import edu.wpi.first.cscore.CvSink;
//import edu.wpi.first.cscore.MjpegServer;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
//import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.sensors.PigeonIMU;

public class RobotContainer {
    private Joystick joystick;
    private final Joystick buttonBoard;
    private PigeonIMU pigeon;

    private DriveSubsystem driveSubsystem;
    private WinchSubsystem linearWinchSubsystem;
    private WinchSubsystem rotationalWinchSubsystem;
    private IntakeSubsystem intakeLinearSubsystem;
    private IntakeSubsystem intakeRotationSubsystem;
    Command closeAuto; 
    Command farAuto;
    SendableChooser<Command> AutoChooser;
    Shuffleboard shuffleboard;
    ShuffleboardTab tab = Shuffleboard.getTab("Main");

    public RobotContainer(){
        //create joystick, drive subsystem, linear winch subsystem, and rotational winch subsystem
        joystick = new Joystick(0);
        buttonBoard = new Joystick(1);
        pigeon = new PigeonIMU(8);

       
        

        driveSubsystem = new DriveSubsystem();
        linearWinchSubsystem = new WinchSubsystem(Linear.leftID, Linear.rightID, Linear.max, Linear.min, Linear.linearController, false, Linear.gearboxRatio);
        rotationalWinchSubsystem = new WinchSubsystem(Rotation.leftID, Rotation.rightID, Rotation.max, Rotation.min, Rotation.rotationController, true, Rotation.gearboxRatio);
        intakeLinearSubsystem = new IntakeSubsystem(Intake.IntakeLinearID);
        intakeRotationSubsystem = new IntakeSubsystem(Intake.IntakeRotationID);
        //linearWinchSubsystem.callibrate(0);
        //rotationalWinchSubsystem.callibrate(RotationalWinchUtil.findRotationalWinchPos(90));

        //create and set default drive command
        driveSubsystem.setDefaultCommand(new DriveCommand(() -> joystick.getRawAxis(1)*-1 * 0.25, () -> joystick.getRawAxis(0), driveSubsystem)); 
        linearWinchSubsystem.setDefaultCommand(new MotionMagicCommandHang(() -> linearWinchSubsystem.getleftPostition(), linearWinchSubsystem, 0, 0));
                //create forward and backward rotational winch command
        initButton(); 
        initChooser();
        initCam();
    }
 
    
    public Command getAutoCommand(){
        return new SequentialCommandGroup(
            new AutoCalibrate(linearWinchSubsystem, 4, -.1, -0.1), 
            //new SetYawCommand(true, pigeon),
            new WinchCommand(.15, 8, linearWinchSubsystem), 
            new WinchCommand(.15, RotationalWinchUtil.findRotationalWinchPos(78), rotationalWinchSubsystem), 
            new WinchCommand(.5, RotationalWinchUtil.findRotationalWinchPos(71), rotationalWinchSubsystem),
            new WaitCommand(1),
            new ParallelCommandGroup(
                new WinchCommand(Rotation.power, RotationalWinchUtil.findRotationalWinchPos(90), rotationalWinchSubsystem),
                new AutoDrive(.2, (9*12), driveSubsystem), 
                new WinchCommand(Linear.power*2, 0, linearWinchSubsystem)
            ),
            //new AutoDrive(.5, (2*12), driveSubsystem), 
            new AutoDrive(.7, -42, driveSubsystem)
        );
        
    }

    public Command getAltAutoCommand() { 
        return new SequentialCommandGroup( 
            new AutoCalibrate(linearWinchSubsystem, 4, -.1, -0.1),
            new WinchCommand(.1, RotationalWinchUtil.findRotationalWinchPos(105), rotationalWinchSubsystem),
            new WaitCommand(3),
            new AutoDrive(.25, (-4*12), driveSubsystem),
            new AutoTurn(.15, 45, driveSubsystem, pigeon),
            new AutoDrive(.25, (-9), driveSubsystem),
            //new SetYawCommand(true, pigeon),
            new ParallelCommandGroup(
                new MotionMagicCommand(8, linearWinchSubsystem, 15, 15),
                new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(78), rotationalWinchSubsystem, 15, 15)
            ),
            new WinchCommand(.5, RotationalWinchUtil.findRotationalWinchPos(74), rotationalWinchSubsystem),
            new WaitCommand(.1),
            new ParallelCommandGroup(
                new WinchCommand(Rotation.power, RotationalWinchUtil.findRotationalWinchPos(90), rotationalWinchSubsystem),
                new AutoDrive(.2, (2*12), driveSubsystem),
                new WinchCommand(Linear.power*2, 0, linearWinchSubsystem)
            ),
            new AutoTurn(.15, -45, driveSubsystem, pigeon),
            new ParallelCommandGroup(
                new AutoDrive(.3, (6*12), driveSubsystem),
                new IntakeCommand(-1, intakeLinearSubsystem) 
            )
            
        );
    }

    public void initButton(){
        new JoystickButton(joystick, 6).whileHeld(new WinchCommand(Rotation.power,  rotationalWinchSubsystem));
        new JoystickButton(joystick, 4).whileHeld(new WinchCommand(Rotation.power*-1, rotationalWinchSubsystem));
        //create forward and backward linear winch command
        new JoystickButton(joystick, 5).whileHeld(new WinchCommand(Linear.power, linearWinchSubsystem));
        new JoystickButton(joystick, 3).whileHeld(new WinchCommand(Linear.power*-1, linearWinchSubsystem));
        //create callibrate commands
        //new JoystickButton(joystick, 9).whenPressed(new AutoClimb(linearWinchSubsystem, rotationalWinchSubsystem));
        new JoystickButton(joystick, 9).whenPressed(new MotionMagicAutoClimb(linearWinchSubsystem, rotationalWinchSubsystem));
        new JoystickButton(joystick, 10).whenPressed(new MotionMagicAutoClimb(linearWinchSubsystem, rotationalWinchSubsystem).new nextRung(rotationalWinchSubsystem, linearWinchSubsystem));
        new JoystickButton(joystick, 8).whenPressed(new MotionMagicAutoClimb(linearWinchSubsystem, rotationalWinchSubsystem).getExtend());

        //new JoystickButton(joystick, 8).whenPressed(new HangWait(2, linearWinchSubsystem));

        JoystickButton up =  new JoystickButton(buttonBoard, 1);
        JoystickButton down =  new JoystickButton(buttonBoard, 2);

        new JoystickButton(buttonBoard, 12).whileHeld(new OneMotorCommand(linearWinchSubsystem.getLeftTanlonFX(), () -> up.get(), () -> down.get()));
        new JoystickButton(buttonBoard, 13).whileHeld(new OneMotorCommand(linearWinchSubsystem.getRightTanlonFX(), () -> up.get(), () -> down.get()));
        new JoystickButton(buttonBoard, 14).whileHeld(new OneMotorCommand(rotationalWinchSubsystem.getLeftTanlonFX(), () -> up.get(), () -> down.get()));
        new JoystickButton(buttonBoard, 15).whileHeld(new OneMotorCommand(rotationalWinchSubsystem.getRightTanlonFX(), () -> up.get(), () -> down.get()));

        JoystickButton callibrateSwitch =  new JoystickButton(buttonBoard, 16);

        new JoystickButton(buttonBoard, 3).whenPressed(new Callibrate(linearWinchSubsystem, 0, () -> callibrateSwitch.get()));
        new JoystickButton(buttonBoard, 4).whenPressed(new Callibrate(rotationalWinchSubsystem, RotationalWinchUtil.findRotationalWinchPos(90), () -> callibrateSwitch.get()));

        // new JoystickButton(joystick, 11).whenPressed(new MotionMagicCommand(5, linearWinchSubsystem, 15, 20));
        // new JoystickButton(joystick, 12).whenPressed(new MotionMagicCommand(15, linearWinchSubsystem, 15, 20));

        new JoystickButton(joystick, 7).whenPressed(new AutoTurn(.2, -45, driveSubsystem, pigeon)); //angle to hangar

        new JoystickButton(joystick, 1).whileHeld(new IntakeCommand(-1, intakeLinearSubsystem));   //intake in
        new JoystickButton(joystick, 2).whileHeld(new IntakeCommand(1, intakeLinearSubsystem)); //intake out
        
        new JoystickButton(joystick, 12).whileHeld(new IntakeCommand(.25, intakeRotationSubsystem));  //intake up?
        new JoystickButton(joystick, 11).whileHeld(new IntakeCommand(-.25, intakeRotationSubsystem));  //intake down?
    }

    public void initChooser(){
        closeAuto = getAutoCommand();
        farAuto = getAltAutoCommand();
        AutoChooser = new SendableChooser<>();
        AutoChooser.setDefaultOption("Close Auto", closeAuto);
        AutoChooser.addOption("Away Auto", farAuto);
        tab.add("AutoSender", AutoChooser);
    }

    public Command getAutonomousCommand() {
        return AutoChooser.getSelected();
    } 

    public void initCam() {
        // Get the back camera plugged into the RIO

        UsbCamera cam = CameraServer.startAutomaticCapture("Camera", 0);
        cam.setResolution(160, 120);
        cam.setFPS(20);
        // MjpegServer cam0Stream = new MjpegServer("Top Camera", 1185);
        // cam0Stream.setSource(cam);
        tab.add("Camera", cam);
        // UsbCamera camera = new UsbCamera("MainCamera", 0);
        // MjpegServer mjpegServer = new MjpegServer("mjpegmain", 1185);
        // CvSink cvsink = new CvSink("cvmain");










        // UsbCamera cam0 = CameraServer.startAutomaticCapture("Abc", 0);
        // cam0.setResolution(160, 120);
        // cam0.setFPS(20);

        // MjpegServer cam0Stream = new MjpegServer("Top Camera", 1185);
        // cam0Stream.setSource(cam0);
        // cam0Stream.setResolution(160, 120);
        // cam0Stream.setFPS(20);
        // cam0Stream.setCompression(70);
        // ShuffleboardTab tab = Shuffleboard.getTab("Main");
        // tab.add("Camera", cam0);

        
        //switchCamera(1);

        //UsbCamera topCamera = CameraServer.startAutomaticCapture(0);
        //topCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        //UsbCamera frontCamera = CameraServer.startAutomaticCapture(1);
        //topCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);

        // // Feed that back camera into a new stream so we can add compression
        // MjpegServer topCameraStream = new MjpegServer("Top Camera", 1185);
        // topCameraStream.setSource(topCamera);
        // // Compress the stream, set its resolution, and set its framerate along with the camera's
        // topCameraStream.setCompression(Constants.Camera.kCameraCompression);
        // topCamera.setResolution(Constants.Camera.kCameraResolution.getX(), Constants.Camera.kCameraResolution.getY());
        // topCameraStream.setResolution(Constants.Camera.kCameraResolution.getX(), Constants.Camera.kCameraResolution.getY());
        // topCamera.setFPS(Constants.Camera.kCameraFPS);
        // topCameraStream.setFPS(Constants.Camera.kCameraFPS);

        // Get the back camera plugged into the RIO
        //frontCamera.setConnectionStrategy(ConnectionStrategy.kKeepOpen);
        //VideoSink server = CameraServer.getServer();
       

        // // Feed that back camera into a new stream so we can add compression
        // MjpegServer frontCameraStream = new MjpegServer("Front Camera", 1186);
        // frontCameraStream.setSource(frontCamera);
        // // Compress the stream, set its resolution, and set its framerate along with the camera's
        // frontCameraStream.setCompression(Constants.Camera.kCameraCompression);
        // frontCamera.setResolution(Constants.Camera.kCameraResolution.getX(), Constants.Camera.kCameraResolution.getY());
        // frontCameraStream.setResolution(Constants.Camera.kCameraResolution.getX(), Constants.Camera.kCameraResolution.getY());
        // frontCamera.setFPS(Constants.Camera.kCameraFPS);
        // frontCameraStream.setFPS(Constants.Camera.kCameraFPS);
        
    }


}


/*

have equilize command that's called in end in winchCommand

 */