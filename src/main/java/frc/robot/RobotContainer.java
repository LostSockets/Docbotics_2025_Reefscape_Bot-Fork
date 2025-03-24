// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ManageLimeLightCMD;
import frc.robot.commands.MoveArmCMD;
import frc.robot.commands.SwerveJoystickCmd;

import edu.wpi.first.cameraserver.CameraServer;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ArmSub;

import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LimelightSub;

import com.fasterxml.jackson.core.io.IOContext;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;



public class RobotContainer {

  // private final CommandXboxController m_driverController =
  //     new CommandXboxController(OIConstants.kDriverControllerPort);


  private final SwerveSub swerveSub =  new SwerveSub();
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
        () -> driverJoyStick.getRawButton(OIConstants.kOrientToTargetIdx)
        )
      ); // by defualt will work on fields reference frame
      
    limelightSub.setDefaultCommand(
      new ManageLimeLightCMD(limelightSub)
    );

    configureBindings();
  }


  private void configureBindings() {
    //new JoystickButton(driverJoyStick, OIConstants.kMoveArmIdx ).whileTrue(new MoveArmCMD(armsub));


    
  }


  public Command getAutonomousCommand() {


    return new PathPlannerAuto("Auto_driveForwardAndMoveArm");


  }
}