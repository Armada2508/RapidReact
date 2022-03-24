package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WinchSubsystem;
import edu.wpi.first.wpilibj.Timer;

public class AutoCalibrate extends CommandBase {
    private WinchSubsystem subsystem;
    private double maxTime;
    private double calibrateTo;
    private double power;
    private Timer timer;

    public AutoCalibrate(WinchSubsystem subsystem, double maxTime, double calibrateTo, double power){
        this.subsystem = subsystem;
        this.maxTime = maxTime;
        this.calibrateTo = calibrateTo;
        this.power = power;
        timer = new Timer();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void initialize() {
        timer.reset();
        timer.start();
        subsystem.setPower(power, power);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.callibrate(calibrateTo);
        subsystem.setPower(0, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (Math.abs(subsystem.getVelocity()) < 0.1 || timer.get() >= maxTime) && timer.get() > .5;
    } 
    
}
