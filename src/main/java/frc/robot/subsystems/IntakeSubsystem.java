package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;


public class IntakeSubsystem extends SubsystemBase{
    private int motorID;
    private WPI_TalonFX IntakeMotor; 
    public IntakeSubsystem(int motorID) {
        this.motorID = motorID;
        IntakeMotor = new WPI_TalonFX(motorID);
    }

    @Override
    public void periodic(){

    }

    public void setPower(double power) {
        IntakeMotor.set(power);
    }
    
}
