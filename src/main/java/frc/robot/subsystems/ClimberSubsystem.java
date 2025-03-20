package frc.robot.subsystems;

//import java.util.Set;

//import com.ctre.phoenix.motorcontrol.ControlMode;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.FeedbackDevice;
//import com.ctre.phoenix.motorcontrol.InvertType;
//import com.ctre.phoenix.motorcontrol.NeutralMode;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {

    private final SparkMax climberMotor1 = new SparkMax(Constants.ClimberConstants.kClimberMotorPort1, MotorType.kBrushed);
    private final SparkMax climberMotor2 = new SparkMax(Constants.ClimberConstants.kClimberMotorPort2, MotorType.kBrushed);
    private final RelativeEncoder elevatorEncoder = climberMotor1.getEncoder();


    public double getEncoderMeters() {
        return (((RelativeEncoder) elevatorEncoder).getPosition());
      }

    public ClimberSubsystem () {
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ClimberEncoder Value",  getEncoderMeters());

    }

    public void setMotor(double speed) {
        //armPivotMotorFollow.follow(armPivotMotorLead);
        SmartDashboard.putNumber("Climber speed", speed);
        climberMotor1.set(-speed);
        climberMotor2.set(speed);
    }

}