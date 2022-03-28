package frc.robot.commands;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Lib.Encoder;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.Constants;


public class MotionMagicCommand extends CommandBase{
    private final int timeout = 30;
    private final int PIDslotIndex = 0;
    private final int PIDLoopIndex = 0;
    private final int framePeriod = 10;
    private final double kF = 0;
    private final double kP = .2;
    private final double kI = 0;
    private final double kD = 0;

    private WPI_TalonFX motor;
    private double pos;


    public MotionMagicCommand(double pos, WPI_TalonFX motor, WinchSubsystem s){
        this.motor = motor;
        this.pos = pos;
        addRequirements(s);


        motor.configFactoryDefault();
        
        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDslotIndex, timeout);
        //config deadband?
        //motor inverted

        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, framePeriod, timeout);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, framePeriod, timeout);

        motor.configNominalOutputForward(0);
        motor.configNominalOutputReverse(0);
        motor.configPeakOutputForward(.3);
        motor.configPeakOutputReverse(-.3);

        motor.selectProfileSlot(PIDslotIndex, PIDLoopIndex);
        motor.config_kF(PIDslotIndex, kF);
        motor.config_kP(PIDslotIndex, kP);
        motor.config_kI(PIDslotIndex, kI);
        motor.config_kD(PIDslotIndex, kD);

        motor.configMotionCruiseVelocity(Encoder.fromVelocity(20, Constants.Winch.encoderUnits, Constants.Rotation.gearboxRatio, Constants.Winch.diameter, .100));
        motor.configMotionAcceleration(Encoder.fromVelocity(5, Constants.Winch.encoderUnits, Constants.Rotation.gearboxRatio, Constants.Winch.diameter, .100));

        //motor.setSelectedSensorPosition(0);
    }

    @Override
    public void initialize() {
        motor.set(ControlMode.MotionMagic, Encoder.fromDistance(pos, Constants.Winch.encoderUnits, Constants.Rotation.gearboxRatio, Constants.Winch.diameter));
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
       
    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        motor.set(ControlMode.PercentOutput, 0);
        
    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return false;
    }


    
}
