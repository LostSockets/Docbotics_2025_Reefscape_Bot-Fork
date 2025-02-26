// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OIConstants;
import frc.robot.commands.ElevateIntakeToSetpointCMD;
import frc.robot.commands.IdleIntakeHeightCMD;
import frc.robot.commands.IdlePitchIntakeAngleCMD;
import frc.robot.commands.IntakeCoralCMD;
import frc.robot.commands.ManageLimeLightCMD;
import frc.robot.commands.PitchIntakeCMD;
import frc.robot.commands.SwerveJoystickCmd;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.CoralIntakeSub;
import frc.robot.subsystems.ElevatorSub;
import frc.robot.subsystems.SwerveSub;
import frc.robot.subsystems.LimelightSub;

import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class RobotContainer {

  // private final CommandXboxController m_driverController =
  // new CommandXboxController(OIConstants.kDriverControllerPort);

  private final SwerveSub swerveSub = new SwerveSub();
  private final CoralIntakeSub coralIntakeSub = new CoralIntakeSub();
  // private final ArmSub armsub = new ArmSub();
  private final LimelightSub limelightSub = new LimelightSub();
  public final ElevatorSub elevatorSub = new ElevatorSub();
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

    /**
     * By default the the elevator will be in Idle state
     * where it just tries to maintain the intake height set point.
     */
     elevatorSub.setDefaultCommand(
     new IdleIntakeHeightCMD(elevatorSub));
     coralIntakeSub.setDefaultCommand(
      new IdlePitchIntakeAngleCMD(coralIntakeSub)
     );


    configureBindings();
  }

  private void configureBindings() {
    // new JoystickButton(driverJoyStick, OIConstants.kMoveArmIdx ).whileTrue(new
    // MoveArmCMD(armsub));

    new JoystickButton(driverJoyStick, 1).whileTrue(
    new ElevateIntakeToSetpointCMD(
    elevatorSub,
    0));
  

    /**Command to reset intake elevator motors */
    Command resetEncodersCommand = new RunCommand(() -> {
      elevatorSub.resetElevatorEncoders();
    }, elevatorSub);
    SmartDashboard.putData("resetEncodersCommand", resetEncodersCommand);

    new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToLevel2Idx).whileTrue(
        new ElevateIntakeToSetpointCMD(
            elevatorSub,
            10));

    // Command scoreL2Reef = new ParallelCommandGroup(
    //     new PitchIntakeCMD(coralIntakeSub, 15),
    //     new ElevateIntakeToSetpointCMD(
    //         elevatorSub,
    //         15));

    /*
     * when consumer intake coral button is pressed, power the intake consumer motor
     * to intake
     * the coral.
     */
   // new JoystickButton(driverJoyStick, OIConstants.kCoralIntakeIdx).whileTrue(new IntakeCoralCMD(coralIntakeSub));

    // * When pitch intake button is pressed, rotate the intake to the desire
    // set-point. */
    new JoystickButton(driverJoyStick, OIConstants.kMoveIntakeToLevel2Idx)
        .whileTrue(new PitchIntakeCMD(coralIntakeSub, 30
        ));
        new JoystickButton(driverJoyStick, 1)
        .whileTrue(new PitchIntakeCMD(coralIntakeSub, 0
        ));
  }

  public Command getAutonomousCommand() {

    return new PathPlannerAuto("Auto_driveForwardAndMoveArm");

  }
}