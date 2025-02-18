// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ManageLimeLightCMD;
import frc.robot.commands.ElevateIntakeToSetpointCMD;
import frc.robot.commands.IdleIntakeHeightCMD;
import frc.robot.commands.SwerveJoystickCmd;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LimelightSub;
import com.pathplanner.lib.commands.PathPlannerAuto;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {

  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OIConstants.kDriverControllerPort);

  private final SwerveSub swerveSub = new SwerveSub();
  // private final ArmSub armsub = new ArmSub();
  private final LimelightSub limelightSub = new LimelightSub();
  private final ElevatorSub elevatorSub = new ElevatorSub();
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
            () -> driverJoyStick.getRawButton(OIConstants.kOrientToTargetIdx))); // by default will work on fields
                                                                                 // reference frame

    limelightSub.setDefaultCommand(
        new ManageLimeLightCMD(limelightSub));

    /**By default the the elevator will be in Idle state
     * where it just tries to maintain the intake height set point.
    */
    elevatorSub.setDefaultCommand(
      new IdleIntakeHeightCMD(elevatorSub));

    configureBindings();
  }

  private void configureBindings() {
    // new JoystickButton(driverJoyStick, OIConstants.kMoveArmIdx ).whileTrue(new
    // MoveArmCMD(armsub));

    // when A is pressed, move intake to level 2.
    new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToLevel2Idx).whileTrue(
        new ElevateIntakeToSetpointCMD(
            elevatorSub,
            0));

        new JoystickButton(driverJoyStick, 3).whileTrue(
              new ElevateIntakeToSetpointCMD(
                  elevatorSub,
                  15));

        RunCommand resetEncoderCommand = new RunCommand( Torun -> elevatorSub.resetElevatorEncoders(), elevatorSub)
        new JoystickButton(driverJoyStick, 4).whileTrue()

  }

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("Auto_driveForwardAndMoveArm");

  }
}