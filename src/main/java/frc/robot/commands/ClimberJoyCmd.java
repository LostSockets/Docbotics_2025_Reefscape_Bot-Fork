
package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberJoyCmd extends Command {
    private final ClimberSubsystem climberSubsystem;
    private final Supplier<Double> speedFunction;


    double realTimeSpeed = 0;

   
    public ClimberJoyCmd(ClimberSubsystem climberSubsystem, Supplier<Double> speedFunction) {
        this.climberSubsystem = climberSubsystem;
        this.speedFunction = speedFunction;
        addRequirements(climberSubsystem);
    }


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  
    if (Math.abs(realTimeSpeed) < 0.05) {
      realTimeSpeed = 0;
    }

//    if (elevatorSubsystem.getEncoderMeters() < Constants.ElevatorConstants.kmaxElevatorEncoderHeight && elevatorSubsystem.getEncoderMeters() > Constants.ElevatorConstants.kminElevatorEncoderHeight) {
//      realTimeSpeed = speedFunction.get() * Constants.ElevatorConstants.kElevatorSpeedPercentage;
//      elevatorSubsystem.setMotor(-realTimeSpeed);
    realTimeSpeed = speedFunction.get() * Constants.ClimberConstants.kClimberSpeedPercentage;

 
  /*   NOT NEEDED - NO ENCODERS!
     if (climberSubsystem.getEncoderMeters() < Constants.ClimberConstants.kmaxClimberEncoderHeight && realTimeSpeed > 0.0)
      climberSubsystem.setMotor(-realTimeSpeed);
    if (climberSubsystem.getEncoderMeters() > Constants.ClimberConstants.kminClimberEncoderHeight && realTimeSpeed < 0.0)
      climberSubsystem.setMotor(-realTimeSpeed);
  */
    //System.out.println("speed = " + realTimeSpeed);
    //System.out.println("encoder = " + ClimberSubsystem.getEncoderMeters());
    }


  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climberSubsystem.setMotor(0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }

}