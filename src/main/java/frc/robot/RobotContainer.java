// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.ManageLimeLightCMD;

import frc.robot.commands.SwerveJoystickCmd;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LimelightSub;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;




public class RobotContainer {


  // configures the different subsystem of the robot
  private final SwerveSub swerveSub =  new SwerveSub();
 // private final ArmSub armsub = new ArmSub();
  private final LimelightSub limelightSub = new LimelightSub();
  private final Joystick driverJoyStick = new Joystick(OIConstants.kDriverControllerPort);






  public RobotContainer() {
    
    

    
    swerveSub.setDefaultCommand(
        new SwerveJoystickCmd(
        swerveSub,
        () -> -driverJoyStick.getRawAxis(OIConstants.kDriverYAxis),
        () -> driverJoyStick.getRawAxis(OIConstants.kDriverXAxis),
        () -> driverJoyStick.getRawAxis(OIConstants.kDriverRotAxis),
        () -> !driverJoyStick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx),
        () -> driverJoyStick.getRawButton(OIConstants.kOrientToTargetIdx)
        )
      ); 
      
    limelightSub.setDefaultCommand(
      new ManageLimeLightCMD(limelightSub)
    );

    configureBindings();
  }

/**  Configure the trigger bindings for certain commands*/
  private void configureBindings() {
    //new JoystickButton(driverJoyStick, OIConstants.kMoveArmIdx ).whileTrue(new MoveArmCMD(armsub));


    
  }

  //** gets the autonmous Command for the robot */
  public Command getAutonomousCommand() {


    return new PathPlannerAuto("Auto_driveForwardAndMoveArm");


  }
}