
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;
import com.google.flatbuffers.Constants;



import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.bridgeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;



public class BridgeSub extends SubsystemBase{
    private SparkMax bridgeMotor = new SparkMax(bridgeConstants.kBridgeMotorPort, MotorType.kBrushless);
    private PIDController bridgeController= new PIDController(
        bridgeConstants.kP,
        bridgeConstants.kI,
        bridgeConstants.kD);

    
    public BridgeSub(){

        
    }




    public SparkMax getMotor(){
        return bridgeMotor;
    }

    public double getGetBridgeEncoderPosition_degrees(){
        return bridgeMotor.getAbsoluteEncoder().getPosition();
    }

    public PIDController getBridgeController(){
        return bridgeController;
    }
    public void setBridgeMotorPower(double power){
        bridgeMotor.set(power);
    }





}