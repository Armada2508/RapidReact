package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.Constants.Rotation;
import frc.robot.Lib.RotationalWinchUtil;
import frc.robot.Constants.Linear;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class MotionMagicAutoClimb extends SequentialCommandGroup{
    private PigeonIMU pigeon;
    
    private Command autoClimb;
    Command x;
    WinchSubsystem l;
    WinchSubsystem r;


    public MotionMagicAutoClimb(WinchSubsystem linear, WinchSubsystem rotation){
        pigeon = new PigeonIMU(8);
        l = linear;
        r = rotation;
        addCommands(    
            new firstRung(rotation, linear),
            new nextRung(rotation, linear),
            new lastRung(rotation, linear)
        );   
    }

    public double getPigeonHeading(){
        return pigeon.getPitch();
    }

    public Command getNextRung(){
        return new nextRung(r, l);
    }

    public Command getExtend(){
        return new Extend(r, l);
    }


    public Command getAutoClimb(){
        return autoClimb;
    }

    public boolean isOnBar(){
        return true;
    }

    public Command getTest(){
        System.out.println("entered test");
        return autoClimb;
    }

    public class Extend extends ParallelCommandGroup{
        public Extend(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new MotionMagicCommand(20, l, 10, 20),
                new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(90), r, 10, 20)
            );
        }
    }

    public class RetractAndLineUpStatArms extends SequentialCommandGroup{
        public RetractAndLineUpStatArms(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new ParallelCommandGroup(
                    new MotionMagicCommand(Linear.min, l, 15, 20),
                    new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(72), r, 4, 4)
                )
            );
        }
    }

    public class getStatArmsOn extends SequentialCommandGroup{
        public getStatArmsOn(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new MotionMagicCommand(Linear.min, l, 15, 30),
                new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(89), r, 10, 30),
                new MotionMagicCommand(6, l, 10, 30)
            );
        }
    }
    //shorter wait time
    public class getStatArmsOn2 extends SequentialCommandGroup{
        public getStatArmsOn2(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new MotionMagicCommand(Linear.min, l, 15, 30),
                new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(89), r, 10, 30),
                new MotionMagicCommand(2, l, 15, 30)
            );
        }
    }

    public class angleExtendAndGetOnBar extends SequentialCommandGroup{
        public angleExtendAndGetOnBar(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(120), r, 15, 30),
                new MotionMagicCommand(Linear.max-1, l, 25, 60),
                new WaitUntilCommand(() -> atMid()),
                new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(100), r, 15, 45),
                new MotionMagicCommand(24, l, 15, 15),
                new WaitUntilCommand(() -> go()),
                new MotionMagicCommand(16, l, 15, 30)
            );
        }
    }

    private double previousPitch = 0;
    private double currentPitch = 0;
    private double temp = 0;


    public boolean atMid(){
        temp = currentPitch;
        currentPitch = pigeon.getPitch();
        previousPitch = temp;

        if ((currentPitch-previousPitch) < .000001 && currentPitch < 15){
            resetVals();
            return true;
        }
        return false;
        //return (currentPitch-previousPitch) < 0 && (currentPitch-previousPitch) > -.1;
    }

    public void resetVals(){
        previousPitch = 0;
        currentPitch = 0;
        temp = 0;
    }


    public boolean go(){
        temp = currentPitch;
        currentPitch = pigeon.getPitch();
        previousPitch = temp;
        if ((currentPitch-previousPitch) > .000001 && currentPitch > 15){
            resetVals();
            return true;
        }
        return false;
    }

    public class firstRung extends SequentialCommandGroup{
        public firstRung(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new RetractAndLineUpStatArms(r, l),
                new getStatArmsOn(r, l)
            );
        }
    }

    public class nextRung extends SequentialCommandGroup{
        public nextRung(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new angleExtendAndGetOnBar(r, l), 
                new RetractAndLineUpStatArms(r, l),
                new getStatArmsOn(r, l)
            );
        }
    }

    public class lastRung extends SequentialCommandGroup{
        public lastRung(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new angleExtendAndGetOnBar(r, l), 
                new RetractAndLineUpStatArms(r, l),
                new getStatArmsOn2(r, l)
            );
        }
    }
}