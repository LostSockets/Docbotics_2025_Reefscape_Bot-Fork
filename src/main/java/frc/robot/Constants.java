// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import javax.swing.undo.StateEdit;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;


/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static final class ModuleConstants{
    public static final double kWheelDiameterMeters = Units.inchesToMeters(4);    
    public static final double kDriveMotorGearRatio = 1 / 6.75 ;
    public static final double kTurningMotorGearRatio =  1 / 12.8;
    public static final double kDriveEncoderRot2Meter = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters;
    public static final double kTurningEncoderRot2Rad = kTurningMotorGearRatio * 2 * Math.PI;
    public static final double kDriveEncoderRPM2MeterPerSec = kDriveEncoderRot2Meter / 60;
    public static final double kTurningEncoderRPM2RadPerSec = kTurningEncoderRot2Rad / 60;
    public static final double kTurning = 0.25; //changed from 0.5
    
  }
   public static final class DriveConstants {

        public static final double kTrackWidth = Units.inchesToMeters(20.5); //changed
        // Distance between right and left wheels
        public static final double kWheelBase = Units.inchesToMeters(28.5); //changed
        // Distance between front and back wheels
        public static final SwerveDriveKinematics kDriveKinematics = new SwerveDriveKinematics(
                new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(kWheelBase / 2, kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, -kTrackWidth / 2),
                new Translation2d(-kWheelBase / 2, kTrackWidth / 2));

        public static final int kFrontLeftDriveMotorPort = 2;
        public static final int kBackLeftDriveMotorPort = 8;
        public static final int kFrontRightDriveMotorPort = 4;
        public static final int kBackRightDriveMotorPort = 6;

        public static final int kFrontLeftTurningMotorPort = 1;
        public static final int kBackLeftTurningMotorPort = 7;
        public static final int kFrontRightTurningMotorPort = 3;
        public static final int kBackRightTurningMotorPort = 5;



        public static final boolean kFrontLeftTurningEncoderReversed = true;
        public static final boolean kBackLeftTurningEncoderReversed = true;
        public static final boolean kFrontRightTurningEncoderReversed = true;
        public static final boolean kBackRightTurningEncoderReversed = true;

        public static final boolean kFrontLeftDriveEncoderReversed = true;
        public static final boolean kBackLeftDriveEncoderReversed = true;
        public static final boolean kFrontRightDriveEncoderReversed = true;
        public static final boolean kBackRightDriveEncoderReversed = true;

        public static final int kFrontLeftDriveAbsoluteEncoderPort = 15;
        public static final int kBackLeftDriveAbsoluteEncoderPort = 13;
        public static final int kFrontRightDriveAbsoluteEncoderPort = 14;
        public static final int kBackRightDriveAbsoluteEncoderPort = 16;

        public static final boolean kFrontLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackLeftDriveAbsoluteEncoderReversed = false;
        public static final boolean kFrontRightDriveAbsoluteEncoderReversed = false;
        public static final boolean kBackRightDriveAbsoluteEncoderReversed = false;

        public static final double kFrontLeftDriveAbsoluteEncoderOffsetRad = 0.206543 * 2 * Math.PI ;
        public static final double kBackLeftDriveAbsoluteEncoderOffsetRad = 0.373169 * 2 * Math.PI;
        public static final double kFrontRightDriveAbsoluteEncoderOffsetRad = 0.743164 * 2 * Math.PI;
        public static final double kBackRightDriveAbsoluteEncoderOffsetRad = 0.809814 * 2 * Math.PI ;

        public static final double kPhysicalMaxSpeedMetersPerSecond = 5;
        public static final double kPhysicalMaxAngularSpeedRadiansPerSecond = 2 * 2 * Math.PI;

        public static final double kTeleDriveMaxSpeedMetersPerSecond = kPhysicalMaxSpeedMetersPerSecond; // was divided by 3;
        public static final double kTeleDriveMaxAngularSpeedRadiansPerSecond = kPhysicalMaxAngularSpeedRadiansPerSecond; // was divided by 4;
        public static final double kTeleDriveMaxAccelerationUnitsPerSecond = 3;
        public static final double kTeleDriveMaxAngularAccelerationUnitsPerSecond = 3;

        public static class autoTargetConstants{
        public static final double autoOrientKp = 0.0035;
          

        }
    }
    
    public static final class OIConstants {
        public static final int kDriverControllerPort = 0;
        public static final int kOperatorJoystickPort = 1;

        public static final int kDriverXAxis = 0;
        public static final int kDriverYAxis = 1;
        public static final int kDriverRotAxis = 4;
  

        public static final int kDriverFieldOrientedButtonIdx = 1; //A button
        public static final int kOrientToTargetIdx = 2; //B button 
        public static final int kDriveGyroResetButtonIdx = 2;
    

        public static final double kDeadband = 0.5;


    }


    public static final class AutoConstants {
      public static final double kMaxSpeedMetersPerSecond = DriveConstants.kPhysicalMaxSpeedMetersPerSecond / 6;
      public static final double kMaxAngularSpeedRadiansPerSecond = //
              DriveConstants.kPhysicalMaxAngularSpeedRadiansPerSecond / 10;
      public static final double kMaxAccelerationMetersPerSecondSquared = 3;
      public static final double kMaxAngularAccelerationRadiansPerSecondSquared = Math.PI / 4;
      public static final double kPXController = 1.5;
      public static final double kPYController = 1.5;
      public static final double kPThetaController = 3;

      public static final TrapezoidProfile.Constraints kThetaControllerConstraints = //
              new TrapezoidProfile.Constraints(
                      kMaxAngularSpeedRadiansPerSecond,
                      kMaxAngularAccelerationRadiansPerSecondSquared);
 
                      public static final class autoCommands{
        public static final String moveArmCMD ="moveArmCMD";
      }

  }


  public static class ClimberConstants {
    public static final int kClimberMotorPort1 = 26;
    public static final int kClimberMotorPort2 = 25;
    public static final Double kClimberSpeedPercentage = 0.5;

    public static final int CLIMBER_DOWN = 4;
    public static final int CLIMBER_UP = 3;
    public static final double CLIMBER_SPEED = 0.5;
  }

  public static class GyroConstants {
    public static final double gyroOffsetScaleFactor = 1.0361;  // 0.96389 if going the other way
    
  }
}