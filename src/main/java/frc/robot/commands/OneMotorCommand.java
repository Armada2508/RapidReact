package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import java.util.function.BooleanSupplier;

public class OneMotorCommand extends CommandBase {
    private TalonFX moterFx;
    private BooleanSupplier up;
    private BooleanSupplier down;

    /**
     * sets current position to zero
     * @param subsystem
     */
    public OneMotorCommand (TalonFX moterFx, BooleanSupplier up,  BooleanSupplier down) {
        this.moterFx = moterFx;
        this.up = up;
        this.down = down;

    }

    @Override
    public void initialize() {
        if(up.getAsBoolean()){
            moterFx.set(ControlMode.PercentOutput, 0.1);
        }
        else if(down.getAsBoolean()){
            moterFx.set(ControlMode.PercentOutput, -0.1);
        }
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if(up.getAsBoolean()){
            moterFx.set(ControlMode.PercentOutput, 0.1);
        }
        else if(down.getAsBoolean()){
            moterFx.set(ControlMode.PercentOutput, -0.1);
        }
        else{
            moterFx.set(ControlMode.PercentOutput, 0);
        }
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        moterFx.set(ControlMode.PercentOutput, 0);
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }
    
}
