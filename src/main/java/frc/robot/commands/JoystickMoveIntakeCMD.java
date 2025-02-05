package frc.robot.commands;

import java.util.function.Supplier;

import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.ArmSub;
import frc.robot.subsystems.ElevatorSub;

public class JoystickMoveIntakeCMD extends Command {
    public final ElevatorSub elevatorSub;
    public final SparkMax primaryLeftElevatorMotor;
    public final SparkMax rightElevatorMotor;
    /** Joystick input to control the elevator. */
    public final Supplier<Double> elevatorPowerJoystickFunction;
    /** used to smooth the joystick input controlling the elevator */
    private final SlewRateLimiter elevatorPowerLimiter; 

    public JoystickMoveIntakeCMD(
        ElevatorSub elevatorSub,
        Supplier<Double> elevatorPowerJoystickFunction) {
        this.elevatorSub = elevatorSub;
        this.primaryLeftElevatorMotor = elevatorSub.getPrimaryLeftElevatorMotor();
        this.rightElevatorMotor = elevatorSub.getRightElevatorMotor();
        this.elevatorPowerJoystickFunction = elevatorPowerJoystickFunction;

        this.elevatorPowerLimiter = new SlewRateLimiter(ElevatorConstants.elevatorJoystickSensitivity);

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
        SmartDashboard.putNumber("elevatorPosition_meters", elevatorSub.getPrimaryElevatorPosition());

        double elevatorPower = elevatorPowerJoystickFunction.get();
        /** if there is a small input in the joystick, do not move the move the elevator */
        elevatorPower = 
        Math.abs(elevatorPower) > ElevatorConstants.elevatorJoystickDeadband ? elevatorPower : 0.0; 

        elevatorPower = elevatorPowerLimiter.calculate(elevatorPower);


        primaryLeftElevatorMotor.set(elevatorPower);
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
         * When current intake position is less than
         * the minimum height to the ground, stop the command.
         */
        if (elevatorSub.getPrimaryElevatorPosition() < ElevatorConstants.minIntakeHeightToGround_Meters) {
            return true;
        }
        return false;
    }
}