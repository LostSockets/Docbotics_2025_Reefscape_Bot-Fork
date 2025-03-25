// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.ClimberConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ClimberCmd;
import frc.robot.commands.ManageLimeLightCMD;
import frc.robot.commands.ResetHeadingCMD;
import frc.robot.commands.SwerveJoystickCmd;



import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LimelightSub;
import frc.robot.subsystems.ClimberSubsystem;

import java.util.stream.Stream;

import com.fasterxml.jackson.core.io.IOContext;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer {

  private final SendableChooser<Command> autoChooser;

  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OIConstants.kDriverControllerPort);


  private final SwerveSub swerveSub =  new SwerveSub();
  private final LimelightSub limelightSub = new LimelightSub();
  private final ClimberSubsystem climberSubsystem = new ClimberSubsystem();

  private final Joystick driverJoyStick = new Joystick(OIConstants.kDriverControllerPort);
  private final Joystick joyOperator = new Joystick(Constants.OIConstants.kOperatorJoystickPort);




  public RobotContainer() {   

    boolean isComp = false; // Change for competitions
    


    //Auto chooser
    autoChooser = AutoBuilder.buildAutoChooserWithOptionsModifier(
      // This will use Commands.none() as default
      // This will only show autos that start with "comp" while at
      // competition as defined by the programmer.
      (stream) -> isComp 
        ? stream.filter(auto -> auto.getName().startsWith("comp"))
        : stream
    );
    //Dashboard key to select desired auto
    SmartDashboard.putData("Auto Chooser",autoChooser);

    // Configure the trigger bindings
    swerveSub.setDefaultCommand(
        new SwerveJoystickCmd(
        swerveSub,
        () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        () -> driverJoyStick.getRawButton(OIConstants.kOrientToTargetIdx)
        )
      ); // by defualt will work on fields reference frame
      

    //Register ALL named commands here!
    NamedCommands.registerCommand("ResetHeadingCMD", new ResetHeadingCMD(swerveSub));
    NamedCommands.registerCommand("ManageLimeLightCMD", new ManageLimeLightCMD(limelightSub));

    limelightSub.setDefaultCommand(
      new ManageLimeLightCMD(limelightSub)
    );

    configureBindings();
  }


  private void configureBindings() {
    //new JoystickButton(driverJoyStick, OIConstants.kMoveArmIdx ).whileTrue(new MoveArmCMD(armsub));
    new JoystickButton(joyOperator, ClimberConstants.CLIMBER_UP).whileTrue(new ClimberCmd(climberSubsystem, -ClimberConstants.CLIMBER_SPEED)); // climber up
    new JoystickButton(joyOperator, ClimberConstants.CLIMBER_DOWN).whileTrue(new ClimberCmd(climberSubsystem, ClimberConstants.CLIMBER_SPEED)); // climber down


    
  }


  public Command getAutonomousCommand() {


    return autoChooser.getSelected(); 
    //return new PathPlannerAuto("Auto_driveForwardAndMoveArm");


  }
}