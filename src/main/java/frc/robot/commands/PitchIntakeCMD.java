package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.CoralIntakeSub;

public class PitchIntakeCMD extends Command {
    private final CoralIntakeSub intakeSub;
    private final SparkMax intakePitcherMotor;
    private final PIDController intakePitchController;

    public PitchIntakeCMD(CoralIntakeSub intakeSub) {
        this.intakeSub = intakeSub;
        this.intakePitcherMotor = intakeSub.getIntakePitcherMotor();
        this.intakePitchController = intakeSub.getIntakePitchController();
        addRequirements(intakeSub);

    }

    @Override
    public void initialize() {

        /* When command starts, stop the intake pitcher */
        intakePitcherMotor.set(0);
        intakePitcherMotor.stopMotor();

        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", true);
    }

    @Override
    public void execute() {
        double currentIntakePosition_degrees = intakePitcherMotor.getEncoder().getPosition();
        // Telemetry.
        SmartDashboard.putData(intakePitchController);
        SmartDashboard.putNumber("intakePitchPositionError_degrees", intakePitchController.getError());
        SmartDashboard.putNumber("intakePitchPosition_degrees", currentIntakePosition_degrees);
        // Pitches intake to set point based on intake pitch controller.
        double output = intakePitchController.calculate(currentIntakePosition_degrees, 14);
        SmartDashboard.putNumber("intakePitcherOutput", output);
        intakePitcherMotor.set(output);
    }

    @Override
    public void end(boolean interrupted) {
        /* when command ends, stop  the intake pitch motor. */
        SmartDashboard.putBoolean("isIntakePitcherCommandRunning", false);
        intakePitcherMotor.set(0);
        intakePitcherMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        /*
         * when the position error of the Intake Pitcher is less
         * than 3 degrees, end the command
         */
        // if (Math.abs(intakePitchController.getError()) <= 3) {
        //     return true;
        // }
        return false;
    }
}