package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;


public class AutoDrive extends CommandBase{
    private double power;
    private double distance;
    private DriveSubsystem subsystem;

    /**
     * @param power power of motors
     * @param distance inches, positive foward, negative backwards
     * @param driveSubsystem
     */ 
    public AutoDrive(double power, double distance, DriveSubsystem driveSubsystem){
        subsystem = driveSubsystem;
        subsystem.callibrate();

        this.power = Math.abs(power);//it's backwards
        this.distance = distance;
        
        addRequirements(subsystem);
    }
    
    
    @Override
    public void initialize() {
        System.out.println("initailzed");
        if(distance > 0){
            subsystem.setPower(power, power);
        }
        if(distance < 0){
            subsystem.setPower(-1*power, -1*power);
        }
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        System.out.println(subsystem.getRightPostition());
        System.out.println(distance);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        subsystem.setPower(0, 0);        
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (distance > 0 && (subsystem.getRightPostition() /*+ subsystem.getleftPostition())/2*/) > distance) || (distance < 0 && (subsystem.getRightPostition() /*+ subsystem.getleftPostition())/2*/) < distance) || power == 0;
    }

}
