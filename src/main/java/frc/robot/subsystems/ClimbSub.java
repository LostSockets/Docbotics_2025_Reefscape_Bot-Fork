
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;



import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;



public class ClimbSub extends SubsystemBase{
    private SparkMax climbMotor = new SparkMax(ClimbConstants.kMotorPort, MotorType.kBrushless);
    private PIDController climbController= new PIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD);

    




    public SparkMax getMotor(){
        return climbMotor;
    }

    public double getGetArmEncoderPosition_degrees(){
        return climbMotor.getAbsoluteEncoder().getPosition();
    }

    public PIDController getClimbController(){
        return climbController;
    }
    public void setArmMotorPower(double power){
        climbMotor.set(power);
    }





}