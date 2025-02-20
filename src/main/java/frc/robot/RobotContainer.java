// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.IntakeCoralCMD;
import frc.robot.commands.ManageLimeLightCMD;
import frc.robot.commands.PitchIntakeCMD;
import frc.robot.commands.SwerveJoystickCmd;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.CoralIntakeSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LimelightSub;


import com.pathplanner.lib.commands.PathPlannerAuto;


import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {

  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OIConstants.kDriverControllerPort);

  private final SwerveSub swerveSub = new SwerveSub();
  private final CoralIntakeSub coralIntakeSub = new CoralIntakeSub();
  // private final ArmSub armsub = new ArmSub();
  private final LimelightSub limelightSub = new LimelightSub();
  private final Joystick driverJoyStick = new Joystick(OIConstants.kDriverControllerPort);

  public RobotContainer() {

    // Configure the trigger bindings
    swerveSub.setDefaultCommand(
        new SwerveJoystickCmd(
            swerveSub,
            () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
            () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
            () -> !driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
            () -> driverJoyStick.getRawButton(OIConstants.kOrientToTargetIdx))); // by defualt will work on fields
                                                                                 // reference frame

    limelightSub.setDefaultCommand(
        new ManageLimeLightCMD(limelightSub));

    configureBindings();
  }

  private void configureBindings() {



    /* when consumer intake coral button is pressed, power the intake consumer motor to intake 
     * the coral.
    */
    new JoystickButton(driverJoyStick, OIConstants.kCoralIntakeIdx).whileTrue(new IntakeCoralCMD(coralIntakeSub));

    // * When pitch intake button is pressed, rotate the intake to the desire
    // set-point. */
    new JoystickButton(driverJoyStick, OIConstants.kPitchIntakeToSetPointIdx)
        .whileTrue(new PitchIntakeCMD(coralIntakeSub));

  }

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("Auto_driveForwardAndMoveArm");

  }
}