
package frc.robot.subsystems;


import java.util.concurrent.atomic.AtomicBoolean;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


public class ArmSub extends SubsystemBase{
    private SparkMax armMotor = new SparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);
    private PIDController armController= new PIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD);

    
    public ArmSub(){

        
    }




    public SparkMax getMotor(){
        return armMotor;
    }

    public double getGetArmEncoderPosition_degrees(){
        return armMotor.getAbsoluteEncoder().getPosition();
    }

    public PIDController getArmController(){
        return armController;
    }
    public void setArmMotorPower(double power){
        armMotor.set(power);
    }



}