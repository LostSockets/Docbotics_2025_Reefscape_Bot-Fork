
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
    private SparkMaxConfig armMotorConfig = new SparkMaxConfig();
    private SparkMax armMotor = new SparkMax(ArmConstants.kArmMotorPort, MotorType.kBrushless);

    
    public ArmSub(){
        //configures armMotors PID Constants, min and max power outputs,
        //max veloctiy and acceleration
        armMotorConfig
        .closedLoop
        .p(0.0005)
        .i(0)
        .d(0)
        .maxOutput(1)
        .minOutput(-1)
        .maxMotion
            .maxVelocity(10000)
            .maxAcceleration(360)
            .allowedClosedLoopError(1);


        armMotor.configure(armMotorConfig,
        ResetMode.kResetSafeParameters, 
        PersistMode.kNoPersistParameters);
        
    }




    public SparkMax getMotor(){
        return armMotor;
    }

    public AbsoluteEncoder getGetArmEncoder(){
        return armMotor.getAbsoluteEncoder();
    }

    
    public void setArmSpeed(double speed){
        armMotor.set(speed);
    }

}