package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSub;
import edu.wpi.first.wpilibj.DutyCycleEncoder;

public class MoveArmCMD extends Command{
    public final ArmSub armSub; 
    public final SparkMax armMotor;
    public final PIDController armController;


    private final int setpoint = 300;
    
    public MoveArmCMD(ArmSub armSub){
        this.armSub = armSub;
        armController = armSub.getPIDController();
        armMotor = armSub.getMotor();
    
        addRequirements(armSub);
        
    }

    @Override
    public void initialize(){
        

        armMotor.stopMotor();
        armMotor.set(0);
        
    }

    
    @Override
    public void execute(){
     double output = armController.calculate(armSub.getGetArmEncoder().getPosition(), setpoint);
         SmartDashboard.putNumber("armPosition_rotation", armSub.getGetArmEncoder().getPosition());
        SmartDashboard.putNumber("armPowerOutput_PercentOfPower",output);
        armSub.setArmSpeed(1);
        
    }
    @Override
    public boolean isFinished(){
        return false;
    }
}