
package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;
import com.google.flatbuffers.Constants;



import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class ArmSub extends SubsystemBase{
    private SparkMax armMotor = new SparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
    private DutyCycleEncoder m_armEncoder = new DutyCycleEncoder(0);
    
    private PIDController armController = new PIDController(
        ArmConstants.kP, 
        ArmConstants.kI, 
        ArmConstants.kD);
    


    public SparkMax getMotor(){
        return armMotor;
    }
        public AbsoluteEncoder getGetArmEncoder(){

        return armMotor.getAbsoluteEncoder();
    }
    public PIDController getPIDController(){
        return armController;
    }
    
    public void setArmSpeed(double speed){
        armMotor.set(speed);
    }

}