package frc.robot.commands;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;
import com.ctre.phoenix.sensors.PigeonIMU;


public class AutoTurn extends CommandBase{
    private PigeonIMU pigeon;
    private double power;
    private DriveSubsystem subsystem;
    private double degrees;
    private double initialDegrees;

    /**
     * @param power power of motors
     * @param degrees degrees, positive right, negative left, when going backwards
     * @param driveSubsystem
     */
    public AutoTurn(double power, double degrees, DriveSubsystem driveSubsystem, PigeonIMU pigeon){
        this.pigeon = pigeon;
        subsystem = driveSubsystem;
        //subsystem.callibrate();

        this.power = Math.abs(power);//it's backwards
        this.degrees = degrees;
        initialDegrees = pigeon.getYaw();
        
        addRequirements(subsystem);
    }
    
    
    @Override
    public void initialize() {
        System.out.println("initailzed");
        initialDegrees = pigeon.getYaw();

        if(degrees > initialDegrees){
            subsystem.setPower(-1*power, power);
        }
        if(degrees < initialDegrees){
            subsystem.setPower(power, -1*power);
        }
    }

    public double getYaw() {
        return pigeon.getYaw();
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // System.out.println(pigeon.getYaw() + "\t" + degrees);
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        // System.out.println("DONE");
        // System.out.println(getYaw() + "\t" + degrees);
        subsystem.setPower(0, 0);        
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return (degrees > initialDegrees && (pigeon.getYaw()) > degrees) || (degrees < 0 && (pigeon.getYaw()) < degrees) || power == 0;
    }

}
