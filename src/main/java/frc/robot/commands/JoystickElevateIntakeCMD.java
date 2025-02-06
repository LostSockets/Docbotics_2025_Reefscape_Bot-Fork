package frc.robot.commands;

import java.util.function.Supplier;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.subsystems.ElevatorSub;

public class JoystickElevateIntakeCMD extends Command {
    public final ElevatorSub elevatorSub;
    public final SparkMax primaryLeftElevatorMotor;
    public final SparkMax rightElevatorMotor;
    /** Joystick input to control the elevator. */
    public final Supplier<Double> elevatorPowerJoystickFunction;
    /** used to smooth the joystick input controlling the elevator */
    private final SlewRateLimiter elevatorPowerLimiter;

    public JoystickElevateIntakeCMD(
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

        double desiredElevatorPower = elevatorPowerJoystickFunction.get();
        /**
         * If there is a small input in the joystick, do not move the move the elevator.
         */
        double deadbandAppliedElevatorPower = Math
                .abs(desiredElevatorPower) > ElevatorConstants.elevatorJoystickDeadband ? desiredElevatorPower : 0.0;

        double slewedElevatorPower = elevatorPowerLimiter.calculate(deadbandAppliedElevatorPower);
        /* Send elevator telemetry. */
        SmartDashboard.putNumber("elevatorPosition_meters", elevatorSub.getPrimaryElevatorPosition());
        SmartDashboard.putNumber("elevatorPower", slewedElevatorPower);

        /** Apply applicable joystick inputs. */
        primaryLeftElevatorMotor.set(slewedElevatorPower);
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