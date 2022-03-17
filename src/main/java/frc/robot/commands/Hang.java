package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;
//import edu.wpi.first.math.controller.ElevatorFeedforward;

public class Hang extends CommandBase{
    private WinchSubsystem subsystem;
    private double lPower;
    private double rPower;
    
    public Hang (WinchSubsystem subsystem){
        this.subsystem = subsystem;
        addRequirements(subsystem);
        if(subsystem.getVelocity()>.05 /*.05 inches/second?*/){
            lPower = .1;// Fd/Kt  F = robot weight/2 + springforce*2 = 30 + 12 = 42  d = winch radius = .875   Kt = Kt of falcon * gearbox ration = ** * 20 = 
            rPower = .1;
        }
        else{
            lPower = 0;
            rPower = 0;
        }   
    }

    @Override
    public void initialize() {
        subsystem.setPower(lPower, rPower);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
    
}
