
package frc.robot.subsystems;


import com.revrobotics.spark.SparkMax;



import edu.wpi.first.math.controller.PIDController;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.ClimbConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;



public class ClimbSub extends SubsystemBase{
    /*Climb Motors. */
    private SparkMax primaryLeftclimbMotor = new SparkMax(ClimbConstants.kPrimaryLeftClimbMotorPort, MotorType.kBrushless);
    private SparkMax rightClimbMotor = new SparkMax(ClimbConstants.kRightClimbMotorPort, MotorType.kBrushless);
    
    /*Climb Motor Configurations */
    private SparkMaxConfig primarLeftclimbMotorConfig = new SparkMaxConfig();
    private SparkMaxConfig rightClimbMotorConfig = new SparkMaxConfig();


    
    private PIDController climbController= new PIDController(
        ArmConstants.kP,
        ArmConstants.kI,
        ArmConstants.kD);

    



    /**@return get primary climber motor  */
    public SparkMax getPrimaryClimbMotor(){
        return primaryLeftclimbMotor;
    }

    public double getGetArmEncoderPosition_degrees(){
        return primaryLeftclimbMotor.getAbsoluteEncoder().getPosition();
    }

    public PIDController getClimbController(){
        return climbController;
    }
    public void setArmMotorPower(double power){
        primaryLeftclimbMotor.set(power);
    }





}