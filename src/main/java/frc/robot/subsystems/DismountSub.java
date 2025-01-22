package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.google.flatbuffers.Constants;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class DismountSub extends SubsystemBase{
    private SparkMax dismountMotor = new SparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);

    public SparkMax getMotor(){
        return dismountMotor;
    }

    public void setDisMountMotorSpeed(double speed){
        dismountMotor.set(speed);
    }

}