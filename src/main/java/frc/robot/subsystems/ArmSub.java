
package frc.robot.subsystems;


import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkMax;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


public class ArmSub extends SubsystemBase{
    private SparkBaseConfig armMotorConfig;
    private SparkMax armMotor = new SparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);

    
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