package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.IntakeSubsystem;

public class IntakeCommand extends CommandBase {
    private double power; //between 1 and -1
    private IntakeSubsystem subsystem; 

    public IntakeCommand(double power, IntakeSubsystem subsystem){
        this.power = power;
        this.subsystem = subsystem;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        subsystem.setPower(power);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.setPower(0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    } 
    
}
