package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import com.ctre.phoenix.sensors.PigeonIMU;

public class SetYawCommand extends CommandBase{
    private PigeonIMU pigeon;
    private boolean SetYaw0;
    
    public SetYawCommand(Boolean SetYaw0, PigeonIMU pigeon){
        this.SetYaw0 = SetYaw0;
        this.pigeon = pigeon;
        //True for set Yaw, False for get Yaw
    }
    
    @Override
    public void initialize() {
        if (SetYaw0 == true) {
            pigeon.setYaw(0);
        }
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
