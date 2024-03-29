package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
//import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.WinchSubsystem;
//import frc.robot.Constants.Rotation;
import frc.robot.Lib.RotationalWinchUtil;
import frc.robot.Constants.Linear;
import frc.robot.Constants.Rotation;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
//import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
//import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class MotionMagicAutoClimb extends SequentialCommandGroup{
    private PigeonIMU pigeon;
    
    private Command autoClimb;
    Command x;
    WinchSubsystem l;
    WinchSubsystem r;
    Command hang;


    public MotionMagicAutoClimb(WinchSubsystem linear, WinchSubsystem rotation){
        pigeon = new PigeonIMU(8);
        l = linear;
        r = rotation;
        hang = new MotionMagicCommandHang(()-> linear.getleftPostition(), linear, 0, 0);
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
                new SequentialCommandGroup(
                    new MotionMagicCommand(8, l, 15, 20),
                    new ParallelRaceGroup(
                        new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(72), r, 4, 4),
                        new MotionMagicCommandHang(()-> l.getleftPostition(), l, 0, 0)
                    ),
                    new MotionMagicCommand(Linear.min, l, 15, 20)
                )
            );
        }
    }

    public class getStatArmsOn extends SequentialCommandGroup{
        public getStatArmsOn(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new MotionMagicCommand(Linear.min, l, 15, 30),
                new ParallelRaceGroup(
                    new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(92), r, 10, 30),
                    new MotionMagicCommandHang(()-> l.getleftPostition(), l, 0, 0)

                ),
                new ParallelRaceGroup(
                    new WaitUntilCommand(() -> arms()),
                    new MotionMagicCommandHang(()-> l.getleftPostition(), l, 0, 0)
                ),
                new MotionMagicCommand(6, l, 10, 30)
            );
        }
    }

    public boolean arms(){
        return pigeon.getPitch()<16;
    }

    public class getStatArmsOn2 extends SequentialCommandGroup{
        public getStatArmsOn2(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new MotionMagicCommand(Linear.min, l, 15, 30),
                new ParallelRaceGroup(
                    new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(92), r, 10, 30),
                    new MotionMagicCommandHang(()-> l.getleftPostition(), l, 0, 0)
                ),
                new MotionMagicCommand(2, l, 15, 30)
            );
        }
    }

    public class angleExtendAndGetOnBar extends SequentialCommandGroup{
        public angleExtendAndGetOnBar(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new ParallelCommandGroup(
                    new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(110), r, 15, 30),
                    new MotionMagicCommand(Linear.max-1, l, 25, 45)
                ),
                new ParallelRaceGroup(
                    new WaitUntilCommand(() -> atMid()),
                    new MotionMagicCommandHang(()-> l.getleftPostition(), l, 0, 0)
                ),
                new ParallelRaceGroup(
                    new MotionMagicCommand(RotationalWinchUtil.findRotationalWinchPos(94), r, 15, 45),
                    new MotionMagicCommandHang(()-> l.getleftPostition(), l, 0, 0)
                ),
                new MotionMagicCommand(24, l, 15, 30),
                new ParallelRaceGroup(
                    new WaitUntilCommand(() -> go()),
                    new MotionMagicCommandHang(()-> l.getleftPostition(), l, 0, 0)
                ),
                new MotionMagicCommand(8, l, 25, 60)
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

        if ((currentPitch-previousPitch) < 0 && currentPitch < Rotation.hangDegrees){
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
        if ((currentPitch-previousPitch) > 0 && currentPitch > Rotation.hangDegrees){
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