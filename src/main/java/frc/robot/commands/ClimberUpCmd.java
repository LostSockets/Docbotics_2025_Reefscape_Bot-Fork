package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants;
import frc.robot.subsystems.ClimberSubsystem;

public class ClimberUpCmd extends Command{

    private final ClimberSubsystem climberSubsystem;
    private final double speed;

    public ClimberUpCmd(ClimberSubsystem climberSubsystem, double speed){
        this.climberSubsystem = climberSubsystem;
        this.speed = -ClimberConstants.CLIMBER_SPEED;
        addRequirements(climberSubsystem);
    }
    
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}
  
    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
      climberSubsystem.setMotor(speed);
      System.out.println("speed" + speed);
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