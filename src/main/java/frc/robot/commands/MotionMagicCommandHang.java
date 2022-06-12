package frc.robot.commands;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.FollowerType;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Lib.Encoder;
import frc.robot.subsystems.WinchSubsystem;
import frc.robot.Constants;

//import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;


public class MotionMagicCommandHang extends CommandBase{
    private final int timeout = 30;
    private final int PIDslotIndex = 0;
    private final int PIDLoopIndex = 0;
    private final int framePeriod = 10;
    private final double kF = 0;
    private final double kP = .5;
    private final double kI = 0.0001;//0.02;//0.003;
    private final double kD = 3;
    // private final double kP = 0.5;
    // private final double kI = 0.02;//0.003;
    // private final double kD = 0.01;//0.02;
    private final double velocity;
    private final double acceleration;

    private WPI_TalonFX motor;
    private WPI_TalonFX motor2;
    private WinchSubsystem s;
    private DoubleSupplier d;

    private double targetPos;


    public MotionMagicCommandHang(DoubleSupplier d, WinchSubsystem s, double velocity, double acceleration){
        this.motor = s.getLeftTanlonFX();
        this.motor2 = s.getRightTanlonFX();
        this.targetPos = d.getAsDouble();
        this.d = d;
        this.s = s;
        this.velocity = velocity;
        this.acceleration = acceleration;

        addRequirements(s);
        

        //motor.configFactoryDefault();
        //motor2.configFactoryDefault();

        //motor2.setInverted(true);


        motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDslotIndex, timeout);
        motor2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, PIDslotIndex, timeout);

        //config deadband?
        //motor.setInverted(true); //inverted

        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, framePeriod, timeout);
        motor2.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, framePeriod, timeout);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, framePeriod, timeout);
        motor2.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, framePeriod, timeout);


        motor.configNominalOutputForward(0);
        motor2.configNominalOutputForward(0);
        motor.configNominalOutputReverse(0);
        motor2.configNominalOutputReverse(0);
        motor.configPeakOutputForward(1);
        motor2.configPeakOutputForward(1);
        motor.configPeakOutputReverse(-1);
        motor2.configPeakOutputReverse(-1);


        motor.selectProfileSlot(PIDslotIndex, PIDLoopIndex);
        motor2.selectProfileSlot(PIDslotIndex, PIDLoopIndex);

        motor.config_kF(PIDslotIndex, kF);
        motor2.config_kF(PIDslotIndex, kF);

        motor.config_kP(PIDslotIndex, kP);
        motor2.config_kP(PIDslotIndex, kP);

        motor.config_kI(PIDslotIndex, kI);
        motor2.config_kI(PIDslotIndex, kI);

        motor.config_kD(PIDslotIndex, kD);
        motor2.config_kD(PIDslotIndex, kD);



        motor.configMotionCruiseVelocity(Encoder.fromVelocity(velocity, Constants.Winch.encoderUnits, s.getGearBoxRatio(), Constants.Winch.diameter, .100));
        motor2.configMotionCruiseVelocity(Encoder.fromVelocity(velocity, Constants.Winch.encoderUnits, Constants.Linear.gearboxRatio, Constants.Winch.diameter, .100));

        System.out.println(motor.configMotionAcceleration(Encoder.fromVelocity(acceleration, Constants.Winch.encoderUnits, Constants.Linear.gearboxRatio, Constants.Winch.diameter, .100)));
        motor2.configMotionAcceleration(Encoder.fromVelocity(acceleration, Constants.Winch.encoderUnits, Constants.Linear.gearboxRatio, Constants.Winch.diameter, .100));

        //motor.setSelectedSensorPosition(0);

    }

    @Override
    public void initialize() {
        this.targetPos = d.getAsDouble();

        motor.set(ControlMode.MotionMagic, Encoder.fromDistance(targetPos, Constants.Winch.encoderUnits, s.getGearBoxRatio(), Constants.Winch.diameter));
       // motor2.follow(motor, FollowerType.AuxOutput1);
        motor2.set(ControlMode.MotionMagic, Encoder.fromDistance(targetPos, Constants.Winch.encoderUnits, s.getGearBoxRatio(), Constants.Winch.diameter));
        motor.setIntegralAccumulator(0);
        motor2.setIntegralAccumulator(0);
    }
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        //System.out.println(motor.getIntegralAccumulator()  + "  " + motor.getErrorDerivative());
       // motor2.follow(motor, FollowerType.AuxOutput1);

    }
  
    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        motor.set(ControlMode.PercentOutput, 0);
        motor2.set(ControlMode.PercentOutput, 0);

    }
  
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        //return false;
        //return Encoder.toDistance(motor.getSelectedSensorPosition(), Constants.Winch.encoderUnits, Constants.Linear.gearboxRatio, Constants.Winch.diameter) >= targetPos - 0.3 && Encoder.toDistance(motor.getSelectedSensorPosition(), Constants.Winch.encoderUnits, Constants.Linear.gearboxRatio, Constants.Winch.diameter) <= targetPos + 0.3) {
      
        return false;
        
    }


    
}
