
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;




import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class CoralIntakeSub extends SubsystemBase{
    private SparkMax armMotor = new SparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);




    public SparkMax getMotor(){
        return armMotor;
    }



    public void setIntakeMotorPower(double power){
        armMotor.set(power);
    }





}