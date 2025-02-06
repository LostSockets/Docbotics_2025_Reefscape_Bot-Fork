package frc.robot.commands;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;

public class ElevateIntakeToSetpointCMD extends Command {
    public final ElevatorSub elevatorSub;
    public final SparkMax primaryLeftElevatorMotor;
    public final SparkMax rightElevatorMotor;
    public final PIDController elevatorController;

    public ElevateIntakeToSetpointCMD(ElevatorSub elevatorSub) {
        this.elevatorSub = elevatorSub;
        this.primaryLeftElevatorMotor = elevatorSub.getPrimaryLeftElevatorMotor();
        this.rightElevatorMotor = elevatorSub.getRightElevatorMotor();
        elevatorController = elevatorSub.getElevatorController();
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

        SmartDashboard.putBoolean("isElevatorCommandRunning", true);
    }

    @Override
    public void execute() {
        /* Send elevator telemetry */
        SmartDashboard.putData(elevatorController);
        SmartDashboard.putNumber("elevatorPositionError_meters", elevatorController.getError());
        SmartDashboard.putNumber("elevatorPosition_meters", elevatorSub.getPrimaryElevatorPosition());
        // Drive elevator Motor to set-point based on elevator controller.
        double output = elevatorController.calculate(elevatorSub.getPrimaryElevatorPosition(), 0.5);
        primaryLeftElevatorMotor.set(output);
    }

    /**
     * When elevator command ends
     * stop all motors;
     */
    @Override
    public void end(boolean interrupted) {
        SmartDashboard.putBoolean("isElevatorCommandRunning", false);
        primaryLeftElevatorMotor.set(0);
        primaryLeftElevatorMotor.stopMotor();
    }

    @Override
    public boolean isFinished() {
        /*
         * When current elevator position is less than 0.2m away
         * end the command.
         */
        if (Math.abs(elevatorController.getError()) <= 0.2) {
            return true;
        }
        return false;
    }
}