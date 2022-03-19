package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.Constants.Rotation;
import frc.robot.Constants.Winch;
import frc.robot.Lib.RotationalWinchUtil;
import frc.robot.Constants.Linear;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandGroupBase;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj2.command.WaitUntilCommand;


public class AutoClimb extends SequentialCommandGroup{
    private PigeonIMU pigeon;
    
    private Command autoClimb;
    Command x;
    WinchSubsystem l;
    WinchSubsystem r;


    public AutoClimb(WinchSubsystem linear, WinchSubsystem rotation){
        pigeon = new PigeonIMU(10);
        l = linear;
        r = rotation;
        addCommands(    
            new firstRung(rotation, linear),
            new PrintCommand("first rung"),
            //new WaitCommand(3),
            new nextRung(rotation, linear),
            new PrintCommand("second rung"),
            //new WaitCommand(3),
            new nextRung(rotation, linear), 
            //new WaitCommand(3),
            new PrintCommand("third rung")
        ); 
        
       
    }
    public Command getNextRung(){
        return new nextRung(r, l);
    }

    public Command getx(){
        return x;
    }  

    /*since pendulums have constant period despite differing hieghts, 
    if we extend the arms in the correct place in the period each time
    then it will always do the same.
    Ei.if we know the period is 2 seconds and it takes 1 second to extend arms all the way
    if we want to have the arms reach the max when the angle is greatest to the left of the y axis
    (which would be logical so we clear the bar and catch it)
    then we should always start extending the arms when the angle is greatest to the right of the y axis
    if it takes .7 seconds to raise the arms then we should extend them .3 seconds after the max
    we can find the max by when velocity is 0 or angle changes fron increasing to decreasing and pitch>0
    */
    //pigeon.getPitch();

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

    public class RetractAndLineUpStatArms extends SequentialCommandGroup{
        public RetractAndLineUpStatArms(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new WinchCommand(Linear.power, 6, l),
                //new WaitCommand(2),
                new WinchCommand(Rotation.power, RotationalWinchUtil.findRotationalWinchPos(75), r), 
                new WinchCommand(Rotation.power, RotationalWinchUtil.findRotationalWinchPos(75), r), 
                //new WaitCommand(2),
                //new WinchCommand(Linear.power, 0, l),
                new PrintCommand("retract and lining up stat arms")
            );
        }
    }

    public class getStatArmsOn extends SequentialCommandGroup{
        public getStatArmsOn(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new WinchCommand(Linear.power, Linear.min, l),
                new WinchCommand(Linear.power, Linear.min, l),
                new WinchCommand(Rotation.power, RotationalWinchUtil.findRotationalWinchPos(93), r), //get stat arms on
                //new WaitCommand(2),
                new WinchCommand(Linear.power, 8, l),
                //new WaitCommand(2),
                new PrintCommand("got stat arms on")
            );
        }
    }

    public class angleExtendAndGetOnBar extends SequentialCommandGroup{
        public angleExtendAndGetOnBar(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new WinchCommand(Rotation.power, RotationalWinchUtil.findRotationalWinchPos(110), r),
                //new WaitCommand(2),
                new WinchCommand(Linear.power, Linear.max, l),//extend all the way
                //new WaitCommand(2),
                new WinchCommand(Rotation.power, RotationalWinchUtil.findRotationalWinchPos(105), r),//rotate so on bar
                //new WaitCommand(2),
                new WinchCommand(Linear.power, 19, l) //retract a little so hook on
                ////new WaitCommand(2),
                
                //new PrintCommand("extend and on next bar")
            );
            //addCommands(new WinchCommand(Linear.power, 23, l));
            //addCommands(new WinchCommand(Rotation.power, RotationalWinchUtil.findRotationalWinchPos(110), r));
        }
    }

    public class firstRung extends SequentialCommandGroup{
        public firstRung(WinchSubsystem r, WinchSubsystem l){
            addCommands(
                new RetractAndLineUpStatArms(r, l),
                //new WaitCommand(2),
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
}