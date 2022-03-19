package frc.robot.commands;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;

public class Callibrate extends CommandBase {
    private WinchSubsystem subsystem;
    private BooleanSupplier b;
    private double val;

    /**
     * sets current position to zero
     * @param subsystem
     */
    public Callibrate (WinchSubsystem subsystem, double val, BooleanSupplier b){
        this.b = b;
        this.subsystem = subsystem;
        this.val = val;

    }

    @Override
    public void initialize() {
        
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(b.getAsBoolean()){
            subsystem.callibrate(val);
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
