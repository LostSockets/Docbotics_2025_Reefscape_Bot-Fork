package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeConsumerSub;
import frc.robot.subsystems.CoralPitcherIntakeSub;

public class powerCoralIntakeCMD extends Command {
    private final CoralIntakeConsumerSub intakeConsumerSub;
    private final SparkMax intakeConsumerMotor;
    private final double power;

    public powerCoralIntakeCMD(CoralIntakeConsumerSub intakeConsumerSub, double power) {
        this.intakeConsumerSub = intakeConsumerSub;
        this.intakeConsumerMotor = intakeConsumerSub.getIntakeConsumerMotor();
        this.power = power;
        addRequirements(intakeConsumerSub);

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
        intakeConsumerMotor.set(power);
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