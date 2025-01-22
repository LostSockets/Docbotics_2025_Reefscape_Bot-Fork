package frc.robot.commands;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystems.ArmSub;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;

public class MoveArmCMD extends Command{
    public final ArmSub armSub; 
    public final SparkMax armMotor;

    public SparkClosedLoopController armController;


    
    public MoveArmCMD(ArmSub armSub){
        this.armSub = armSub;
        armMotor = armSub.getMotor();
        armController = armMotor.getClosedLoopController();
        addRequirements(armSub);
        
    }

    @Override
    public void initialize(){
        

        armMotor.stopMotor();
        armMotor.set(0);
        
    }

    
    @Override
    public void execute(){

        //run arm motor to the setpoint in shuffleboard
        
        double setpoint  =  SmartDashboard.getNumber("armSetpoint", 0);
        
        armController.setReference
        SmartDashboard.putNumber("armSetpoint", setpoint);
         SmartDashboard.putNumber("armPosition_rotation", armSub.getGetArmEncoder().getPosition());

      
        
    }
    @Override
    public boolean isFinished(){
        
        return false;
    }
}