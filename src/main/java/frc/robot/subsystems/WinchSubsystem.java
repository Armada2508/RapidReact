package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.Lib.Encoder;
//import edu.wpi.first.wpilibj.PowerDistribution;

import frc.robot.Constants.Winch;
import edu.wpi.first.math.controller.PIDController;

public class WinchSubsystem extends SubsystemBase{
    private final WPI_TalonFX left;
    private final WPI_TalonFX right;
    private double max;
    private double min;
    private PIDController controller;
    private double gearBoxRatio;

    /**
     * @param ID1
     * @param ID2
     * @param max
     * @param min
     * @param top true if right winch winds clockwwise, false if counterclockwise
     */
    public WinchSubsystem(int ID1, int ID2, double max, double min, PIDController controller, boolean top, double gearBoxRatio){
        this.gearBoxRatio = gearBoxRatio;
        this.controller = controller;
        left = new WPI_TalonFX(ID1);
        right = new WPI_TalonFX(ID2);
        if(top){
            left.setInverted(true);
        }
        else{
            right.setInverted(true);
        }
        
        this.max = max;
        this.min = min;
        brake();
    }

    public PIDController getController(){
        return controller;
    } 
    public WPI_TalonFX getLeftTanlonFX(){
        return left;
    } 
    public WPI_TalonFX getRightTanlonFX(){
        return right;
    } 
    public void brake(){
        left.setNeutralMode(NeutralMode.Brake);
        right.setNeutralMode(NeutralMode.Brake);
    }

    public double getMax(){
        return max;
    }

    public double getMin(){
        return min;
    }

    public void callibrate(){
        left.setSelectedSensorPosition(0);
        right.setSelectedSensorPosition(0);
    }


    public void setPower(double l, double r){
        left.set(l);
        right.set(r);
    }

    @Override
    public void periodic(){
    }

    public double getRightPostition(){
        return Encoder.toDistance(right.getSelectedSensorPosition(), Winch.encoderUnits, gearBoxRatio, Winch.diameter);  
    }
    
    public double getleftPostition(){
        return Encoder.toDistance(left.getSelectedSensorPosition(), Winch.encoderUnits, gearBoxRatio, Winch.diameter); 
    }

    public double getVelocity(){
        return Encoder.toVelocity((left.getSelectedSensorVelocity() + right.getSelectedSensorVelocity())/2, Winch.encoderUnits, gearBoxRatio, Winch.diameter);
    }   
}