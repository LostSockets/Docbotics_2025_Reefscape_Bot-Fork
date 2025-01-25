package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ClimbSub;

public class ClimbCMD extends Command{
    public final ClimbSub climbSub; 
    public final SparkMax climbMotor;
    public final PIDController climbController;


    
    public ClimbCMD(ClimbSub armSub){
        this.climbSub = armSub;
        this.climbMotor = armSub.getMotor();
        climbController = armSub.getClimbController();
        addRequirements(armSub);
        
    }

    @Override
    public void initialize(){
        

        climbMotor.set(0);
        climbMotor.stopMotor();
        
        SmartDashboard.putBoolean("isClimbCommandRunning", true);
    }

    
    @Override
    public void execute(){
        //telemetry
        SmartDashboard.putData(climbController);
        SmartDashboard.putNumber("climbPostionError_degrees",climbController.getError());
        SmartDashboard.putNumber("climbPostion_degrees",climbSub.getGetArmEncoderPosition_degrees());
        //drive arm Motor to setpoint based on arm controller
        double output = climbController.calculate(climbSub.getGetArmEncoderPosition_degrees(), 169);
        climbMotor.set(output);       
    }
    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("isClimbCommandRunning", false);
        climbMotor.set(0);
        climbMotor.stopMotor();
    }
    @Override
    public boolean isFinished(){
        //
        if(Math.abs(climbController.getError()) <= 3){
            return true;
        }
        return false;
    }
}