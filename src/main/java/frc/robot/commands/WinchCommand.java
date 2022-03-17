package frc.robot.commands;

import org.opencv.osgi.OpenCVInterface;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;

public class WinchCommand extends CommandBase{
    private WinchSubsystem subsystem;
    private double lPower;
    private double rPower;
    private PIDController controller;
    private double correction;
    private double targetPos;
    private double max;
    private double min;
    private double leftPos;
    private double rightPos;
    private double oPower;

    /**
     * @param power + - for direction
     * @param winchSubsystem
     */
    public WinchCommand(double power, WinchSubsystem winchSubsystem){
        lPower = power;
        rPower = power;
        subsystem = winchSubsystem;
        correction = 0;
        controller = subsystem.getController();
        if(power>0){
            targetPos = subsystem.getMax() + 1;//set target position above max so it will never be the cause for ending the command
        }
        else{
            targetPos = subsystem.getMin() - 1;
        }
        max = subsystem.getMax();
        min = subsystem.getMin();
        oPower = power;

        addRequirements(subsystem);
    }

    /**
     * @param power magnitude of power to go at
     * @param position to go to
     * @param winchSubsystem
     * will still remain in bounds of subsystem even if position is lower/higher
     */
    public WinchCommand(double power, double position, WinchSubsystem winchSubsystem){
        subsystem = winchSubsystem;
        correction = 0;
        controller = subsystem.getController();

        max = subsystem.getMax();
        min = subsystem.getMin();


        addRequirements(subsystem);
        targetPos = position;

        //tell you if ur dumb
        if (position > max){
            System.err.println("Extend winch distance is greater than the max");
        } else if (position < min){
            System.err.println("Extend winch distance is less than the min");
        }

        //set power appropriately to direction
        if (position > subsystem.getRightPostition()){
            power = Math.abs(power);
        } else {
            power = Math.abs(power) *(-1);
        }

        System.out.println(power);
        oPower = power;
        lPower = power;
        rPower = power;
    }

    @Override
    public void initialize() {
        if (targetPos > subsystem.getRightPostition()){
            oPower = Math.abs(oPower);
        } else {
            oPower = Math.abs(oPower) *(-1);
        }
        lPower = oPower;
        rPower = oPower;
        controller.reset();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        leftPos = subsystem.getleftPostition();
        rightPos = subsystem.getRightPostition();

        correction = controller.calculate(rightPos, leftPos);
        lPower -= correction;
        rPower += correction;
        //System.out.println("rPOs" + rightPos + "  rPower" + rPower + "  lPos" + leftPos + "  lpower" + lPower);
        //System.out.println(controller.getP());
 
        subsystem.setPower(lPower, rPower);
        
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        //lower arm goes to position of higher arm so they are even
      
        //even out sides so it doesn't start uneven next time? done 3/8 - check
        //find average power not origional power so you have closer initial power?
        subsystem.setPower(0, 0);
        subsystem.brake();
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return (lPower>0 && (rightPos>max||rightPos>targetPos||leftPos>max||leftPos>targetPos)) 
      || (lPower<0 && (rightPos<min||leftPos<min||rightPos<targetPos||leftPos<targetPos));
    }
}
