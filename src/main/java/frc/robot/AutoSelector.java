package frc.robot;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DoNothingCMD;






public class AutoSelector {
    private SendableChooser<Command> autoSelector = new SendableChooser<>();

    public AutoSelector(RobotContainer container) {

        /*
         * Add all the options
         */
        autoSelector.setDefaultOption("DO nothing!!!", DoNothingAuto());
        
        autoSelector.addOption("JustMoveForward", JustMoveForward());   
        autoSelector.addOption("MoveForwardAndScore1Coral", MoveForwardAndScore1Coral());     

    }


    public SendableChooser<Command> getAutoChooser() {
        return autoSelector;
    }

    /*
     * Allow calling the autos as a method
     */
    private SequentialCommandGroup DoNothingAuto() {
        return new DoNothingCMD();
    }
    private PathPlannerAuto JustMoveForward() {
        return new PathPlannerAuto("auto_MoveForward");
    }
    private PathPlannerAuto MoveForwardAndScore1Coral() {
        return new PathPlannerAuto("auto_moveForwardAndScore1Coral");
    }
}
