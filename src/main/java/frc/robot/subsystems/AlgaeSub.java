package frc.robot.subsystems;

//IMPORTS
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj.DutyCycle;  // Dunno if we need these
//import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.AlgaeConstants;
import com.revrobotics.spark.SparkLowLevel.MotorType;

//MAIN
public class AlgaeSub extends SubsystemBase{
    private SparkMax leftMotor = new SparkMax(AlgaeConstants.leftMotorPort, MotorType.kBrushless); //idk motor type
    private SparkMax rightMotor = new SparkMax(AlgaeConstants.rightMotorPort, MotorType.kBrushless);

    private PIDController algaeController= new PIDController(
        AlgaeConstants.kP,
        AlgaeConstants.kI,
        AlgaeConstants.kD);


    public AlgaeSub(){} //idk just needs to exist


    public SparkMax getLeftMotor(){
        return leftMotor;
    }

    public SparkMax getRightMotor(){
        return rightMotor;
    }

    public double getGetLeftEncoderPosition_degrees(){
        return leftMotor.getAbsoluteEncoder().getPosition();
    }

    public double getGetRightEncoderPosition_degrees(){
        return rightMotor.getAbsoluteEncoder().getPosition();
    }

    public PIDController getAlgaeController(){
        return algaeController;
    }
    public void setMotorPower(double power){
        leftMotor.set(power);
        rightMotor.set(power*-1); //needs to go backwards
    }





}



