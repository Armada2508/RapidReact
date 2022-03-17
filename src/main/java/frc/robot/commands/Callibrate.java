package frc.robot.commands;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class Callibrate extends CommandBase {
    private WinchSubsystem subsystem;
    private BooleanSupplier b;

    /**
     * sets current position to zero
     * @param subsystem
     */
    public Callibrate (WinchSubsystem subsystem, BooleanSupplier b){
        this.b = b;
        this.subsystem = subsystem;

    }

    @Override
    public void initialize() {
        //had some error on 3/16
        if(b.getAsBoolean()){
            subsystem.callibrate();
        }
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(b.getAsBoolean()){
            subsystem.callibrate();
        }
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
