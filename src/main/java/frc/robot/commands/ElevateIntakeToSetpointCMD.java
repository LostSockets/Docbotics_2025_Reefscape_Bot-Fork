package frc.robot.commands;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.ElevatorSub;

public class ElevateIntakeToSetpointCMD extends Command {
    public final ElevatorSub elevatorSub;
    public final SparkMax primaryLeftElevatorMotor;
    public final SparkMax rightElevatorMotor;
    public final PIDController elevatorController;
    public final double intakeHeightSetPoint_Meters;

    /**
     * 
     * @param elevatorSub elevator subsystem.
     * @param intakeHeightSetPoint_Inches  intake height set point.
     */
    public ElevateIntakeToSetpointCMD(
            ElevatorSub elevatorSub,
            double intakeHeightSetPoint_Inches) {
        this.elevatorSub = elevatorSub;
        this.primaryLeftElevatorMotor = elevatorSub.getPrimaryLeftElevatorMotor();
        this.rightElevatorMotor = elevatorSub.getRightElevatorMotor();
        this.elevatorController = elevatorSub.getElevatorController();
        this.intakeHeightSetPoint_Meters = intakeHeightSetPoint_Inches;
        addRequirements(elevatorSub);

    }

    @Override
    public void initialize() {

        /**
         * When command starts, stop all elevator motors.
         * Right elevator motor will always FOLLOW
         * the left elevator motor in the OPPOSITE direction.
         */
        primaryLeftElevatorMotor.set(0);
        primaryLeftElevatorMotor.stopMotor();

        /** Set the Intake height set point in meters. */
        elevatorSub.setIntakeHeightSetPoint_Inches(intakeHeightSetPoint_Meters);
        SmartDashboard.putBoolean("isElevateIntakeToSetpointCMD", true);


    }

    @Override
    public void execute() {
        /* Send elevator telemetry */
        SmartDashboard.putData(elevatorController);
        SmartDashboard.putNumber("intakeHeightSetPoint", elevatorSub.getIntakeHeightSetPoint_Inches());
        SmartDashboard.putNumber("elevatorPositionError_Inches", elevatorController.getError());
        SmartDashboard.putNumber("elevatorPosition_Inches", elevatorSub.getPrimaryElevatorPosition());
        // Drive elevator Motor to set-point based on elevator controller.
        //AFTER TESTING CHANGE SETPOINT TO THE VARIABLE SETPOINT IN ELEVATORSUB. 
        double output = elevatorController.calculate(elevatorSub.getPrimaryElevatorPosition(), intakeHeightSetPoint_Meters);

        primaryLeftElevatorMotor.set(output);
    }

    /**
     * When elevator command ends
     * stop all motors;
     */
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("isElevateIntakeToSetpointCMD", false);
        primaryLeftElevatorMotor.set(0);
        primaryLeftElevatorMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        /*
         * When current elevator position is less than 0.2" away
         * end the command.
         */
        if (Math.abs(elevatorController.getError()) <= 0.2) {
            return true;
        }
        return false;
    }
}