package frc.robot.commands;


import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ClimbSub;

public class ClimbCMD extends Command{
    public final ClimbSub climbSub; 
    public final SparkMax climbMotor;
    public final PIDController climbController;


    
    public ClimbCMD(ClimbSub climbSub){
        this.climbSub = climbSub;
        this.climbMotor = climbSub.getPrimaryClimbMotor();
        climbController = climbSub.getClimbController();
        addRequirements(climbSub);
        
    }

    @Override
    public void initialize(){
        /*When code starts 
        stop climb motor. */

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
        //drive climb Motor to setpoint based on arm controller
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
        // if climb position is less than 3 units away from 
        // position, end command.
        if(Math.abs(climbController.getError()) <= 3){
            return true;
        }
        return false;
    }
}