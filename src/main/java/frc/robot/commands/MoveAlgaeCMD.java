package frc.robot.commands;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import frc.robot.subsystems.AlgaeSub;

public class MoveAlgaeCMD extends Command{

    public final AlgaeSub algaeSub = new AlgaeSub(); 
    public final SparkMax leftMotor;
    public final SparkMax rightMotor;
    public final PIDController algaeController;


    
    public MoveAlgaeCMD(AlgaeSub armSub){
        //this.algaeSub = algaeSub;
        this.leftMotor = algaeSub.getLeftMotor();
        this.rightMotor = algaeSub.getRightMotor();
        algaeController = algaeSub.getAlgaeController();
        addRequirements(armSub);
        
    }

    @Override
    public void initialize(){
        

        leftMotor.set(0);
        rightMotor.set(0);
        leftMotor.stopMotor();
        rightMotor.stopMotor();
        
        SmartDashboard.putBoolean("isAlgaeCommandRunning", true);
    }

    
    @Override
    public void execute(){
        //telemetry
        SmartDashboard.putData(algaeController);
        SmartDashboard.putNumber("AlgaePIDPostionError_degrees",algaeController.getError());
        SmartDashboard.putNumber("LeftPostion_degrees",algaeSub.getGetLeftEncoderPosition_degrees());
        SmartDashboard.putNumber("RightPostion_degrees",algaeSub.getGetRightEncoderPosition_degrees());
        //drive arm Motor to setpoint based on arm controller
        double leftOutput = algaeController.calculate(algaeSub.getGetLeftEncoderPosition_degrees(), 169);
        leftMotor.set(leftOutput); 
        double rightOutput = algaeController.calculate(algaeSub.getGetRightEncoderPosition_degrees(), 169);
        leftMotor.set(rightOutput);       
    }
    @Override
    public void end(boolean interrupted){
        SmartDashboard.putBoolean("isAlgaeCommandRunning", false);
        leftMotor.set(0);
        leftMotor.stopMotor();
        rightMotor.set(0);
        rightMotor.stopMotor();
    }
    @Override
    public boolean isFinished(){
        //
        if(Math.abs(algaeController.getError()) <= 3){
            return true;
        }
        return false;
    }
}


