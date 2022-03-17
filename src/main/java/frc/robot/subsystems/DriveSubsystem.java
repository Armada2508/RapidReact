package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.PigeonIMU;

import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.Constants.Drive;
import frc.robot.Lib.Encoder;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import com.ctre.phoenix.motorcontrol.ControlMode;
import frc.robot.Lib.util.Util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.geometry.Rotation2d;


public class DriveSubsystem extends SubsystemBase{
    private MotorControllerGroup right;
    private MotorControllerGroup left;
    private WPI_TalonFX TalonFXL; 
    private WPI_TalonFX TalonFXLfollow; 
    private WPI_TalonFX TalonFXR; 
    private WPI_TalonFX TalonFXRfollow;
    private final DifferentialDriveKinematics mKinematics = new DifferentialDriveKinematics(Drive.kTrackWidth); 
    private final DifferentialDriveOdometry odometry;
    private final PigeonIMU mImu = new PigeonIMU(0);



    public DriveSubsystem(){
        TalonFXL = new WPI_TalonFX(Drive.LFID); 
        TalonFXLfollow = new WPI_TalonFX(Drive.LRID); 
        TalonFXR = new WPI_TalonFX(Drive.RFID); 
        TalonFXRfollow = new WPI_TalonFX(Drive.RRID);
        TalonFXL.setInverted(true);
        TalonFXLfollow.setInverted(true);
        TalonFXLfollow.follow(TalonFXL);
        TalonFXRfollow.follow(TalonFXR);
        left =  new MotorControllerGroup(TalonFXL, TalonFXLfollow);
        right = new MotorControllerGroup(TalonFXR, TalonFXRfollow);
        odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));
    }

    @Override
    public void periodic(){

    }

    public void setPower(double leftPower, double rightPower){
        left.set(leftPower);
        right.set(rightPower);
    }
    
    public double getRightPostition(){
        //System.out.println(right.getMotorOutputPercent());   try to see a change in voltage 
        return Encoder.toDistance(TalonFXR.getSelectedSensorPosition(), Drive.encoderUnits, Drive.gearboxRatio, Drive.diameter); 
        
    }
    
    public double getleftPostition(){
        //System.out.println(left.getMotorOutputPercent());  try to see a change in voltage 
        return Encoder.toDistance(TalonFXL.getSelectedSensorPosition(), Drive.encoderUnits, Drive.gearboxRatio, Drive.diameter); 
        
    }
    public void callibrate(){
        TalonFXL.setSelectedSensorPosition(0);
        TalonFXLfollow.setSelectedSensorPosition(0);
        TalonFXR.setSelectedSensorPosition(0);
        TalonFXRfollow.setSelectedSensorPosition(0);
    }

    public void brake(){
        TalonFXL.setNeutralMode(NeutralMode.Brake);
        TalonFXLfollow.setNeutralMode(NeutralMode.Brake);
        TalonFXR.setNeutralMode(NeutralMode.Brake);
        TalonFXRfollow.setNeutralMode(NeutralMode.Brake);
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds() {
        return new DifferentialDriveWheelSpeeds(getVelocityLeft(), getVelocityRight());
    }

    public void setVelocity(DifferentialDriveWheelSpeeds speeds) {
        TalonFXR.set(ControlMode.Velocity, fromVelocity(speeds.rightMetersPerSecond));
        TalonFXL.set(ControlMode.Velocity, fromVelocity(speeds.leftMetersPerSecond));
    }

    public void setVelocity(DifferentialDriveWheelSpeeds speeds, double deadband) {
        if(Util.inRange(speeds.rightMetersPerSecond, deadband)) {
            TalonFXR.set(ControlMode.PercentOutput, 0);
        } else {
            TalonFXR.set(ControlMode.Velocity, fromVelocity(speeds.rightMetersPerSecond));
        }
        if(Util.inRange(speeds.leftMetersPerSecond, deadband)) {
            TalonFXL.set(ControlMode.PercentOutput, 0);
        } else {
            TalonFXL.set(ControlMode.Velocity, fromVelocity(speeds.rightMetersPerSecond));
        }
    }

    public void setVoltage(double voltsL, double voltsR) {
        TalonFXL.setVoltage(voltsL);
        TalonFXR.setVoltage(voltsR);
    }
    //Maybe change bc it's only checking one motor on the right
    public double getVelocityRight() {
        return toVelocity((int)TalonFXR.getSelectedSensorVelocity());
    }

    //Maybe change bc it's only checking one motor on the left
    public double getVelocityLeft() {
        return toVelocity((int)TalonFXL.getSelectedSensorVelocity());
    }


    public double getVelocity() {
        return mKinematics.toChassisSpeeds(getWheelSpeeds()).vxMetersPerSecond;
    }

    public double toVelocity(int velocity) {
        return Encoder.toVelocity(velocity, Drive.kFeedbackConfig.getEpr(), Drive.kFeedbackConfig.getGearRatio(), Drive.diameter);
    }

    public double fromVelocity(double velocity) {
        return Encoder.fromVelocity(velocity, Drive.kFeedbackConfig.getEpr(), Drive.kFeedbackConfig.getGearRatio(), Drive.diameter);
    }

    public Pose2d getPose() {
        return odometry.getPoseMeters();
    }

    public double getHeading() {
        return Util.boundedAngleDegrees(mImu.getFusedHeading());
    }


}
