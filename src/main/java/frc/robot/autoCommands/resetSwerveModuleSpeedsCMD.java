package frc.robot.autoCommands;

import java.util.function.Supplier;

import com.revrobotics.spark.SparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralIntakeConsumerSub;
import frc.robot.subsystems.CoralPitcherIntakeSub;
import frc.robot.subsystems.SwerveSub;

public class resetSwerveModuleSpeedsCMD extends Command {
    private final SwerveSub swerveSub;



    public resetSwerveModuleSpeedsCMD(
        SwerveSub swerveSub ) {
        this.swerveSub = swerveSub;

        addRequirements(swerveSub);

    }



    @Override
    public void execute() {
        swerveSub.stopModules();
    }


    @Override
    public boolean isFinished() {

        return false;
    }
}