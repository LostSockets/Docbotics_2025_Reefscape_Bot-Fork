package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralIntakeSub;

public class IntakeCoralCMD extends Command {
    private final CoralIntakeSub intakeSub;
    private final SparkMax intakeConsumerMotor;

    public IntakeCoralCMD(CoralIntakeSub intakeSub) {
        this.intakeSub = intakeSub;
        this.intakeConsumerMotor = intakeSub.getIntakeConsumerMotor();
        addRequirements(intakeSub);

    }

    @Override
    public void initialize() {

        /* When command starts, stop the intake consumer motor. */
        intakeConsumerMotor.set(0);
        intakeConsumerMotor.stopMotor();

        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", true);
    }

    @Override
    public void execute() {
        /* power intake consumer motor */
        intakeConsumerMotor.set(1);
    }

    @Override
    public void end(boolean interrupted) {
        /* when command ends, stop  the intake consumer motor. */
        intakeConsumerMotor.set(0);
        intakeConsumerMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {

        return false;
    }
}