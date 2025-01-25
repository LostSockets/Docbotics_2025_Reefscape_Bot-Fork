package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.BridgeSub;

public class MoveBridgeCMD extends Command{
    public final BridgeSub bridgeSub; 
    public final SparkMax bridgeMotor;
    public final PIDController bridgeController;


    
    public MoveBridgeCMD(BridgeSub armSub){
        this.bridgeSub = armSub;
        this.bridgeMotor = armSub.getMotor();
        bridgeController = armSub.getBridgeController();
        addRequirements(armSub);
        
    }

    @Override
    public void initialize(){
        

        bridgeMotor.set(0);
        bridgeMotor.stopMotor();
        
        SmartDashboard.putBoolean("isArmCommandRunning", true);
    }

    
    @Override
    public void execute(){
        //telemetry
        SmartDashboard.putData(bridgeController);
        SmartDashboard.putNumber("armPostionError_degrees",bridgeController.getError());
        SmartDashboard.putNumber("armPostion_degrees",bridgeSub.getGetBridgeEncoderPosition_degrees());
        //drive arm Motor to setpoint based on arm controller
        double output = bridgeController.calculate(bridgeSub.getGetBridgeEncoderPosition_degrees(), 169);
        bridgeMotor.set(output);       
    }
    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("isArmCommandRunning", false);
        bridgeMotor.set(0);
        bridgeMotor.stopMotor();
    }
    @Override
    public boolean isFinished(){
        //
        if(Math.abs(bridgeController.getError()) <= 3){
            return true;
        }
        return false;
    }
}