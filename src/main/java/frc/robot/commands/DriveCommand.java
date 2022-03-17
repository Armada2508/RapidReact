package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import java.util.function.DoubleSupplier;
import frc.robot.Constants.Drive;

public class DriveCommand extends CommandBase{

    private DoubleSupplier joystickSpeed;
    private DoubleSupplier joystickTurn;
    private DriveSubsystem subsystem;
    private double speed;
    private double turn;
    private double powerFactor;

    /**
     * @param joystickSpeed
     * @param joystickTurn
     * @param driveSubsystem
     */
    public DriveCommand(DoubleSupplier joystickSpeed, DoubleSupplier joystickTurn, DriveSubsystem driveSubsystem){
        this.joystickSpeed = joystickSpeed;
        this.joystickTurn = joystickTurn;
        subsystem = driveSubsystem;
        addRequirements(subsystem);
    }

    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        speed = joystickSpeed.getAsDouble();
        turn = joystickTurn.getAsDouble()/Drive.turnAdjustment;
        powerFactor = findSpeed((speed - turn), (speed + turn));

        subsystem.setPower(((speed - turn)*powerFactor), ((speed + turn)*powerFactor));
    }

    public double findSpeed(double left, double right){
        double p = 1;

        if(left > 1){
            p = 1/left;
        } 
        else if(right > 1){
            p = 1/right;
        }
        return p;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false; 
    }

}
