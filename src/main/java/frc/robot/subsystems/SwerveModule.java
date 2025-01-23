package frc.robot.subsystems; 

import com.ctre.phoenix6.hardware.CANcoder;




import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.ModuleConstants;

import frc.robot.Constants.DriveConstants;

import com.revrobotics.spark.SparkLowLevel.MotorType;

import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;


public class SwerveModule {
    private final SparkMax driveMotor;
    private final SparkMaxConfig driveMotorConfig = new SparkMaxConfig();

    private final SparkMax turningMotor;
    private final SparkMaxConfig turnMotorConfig = new SparkMaxConfig();
 
 




    private final PIDController turningPidController;

    private final CANcoder absoluteEncoder;
    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;
    

    public SwerveModule( 
        int driveMotorID, 
    int turningMotorID,
     boolean driveMotorReversed, 
    boolean turningMotorReversed, 
    int absoluteEncoderID,
     double absoluteEncoderOffset,
      boolean absoluteEncoderReversed){

        this.absoluteEncoderOffsetRad = absoluteEncoderOffset;
        this.absoluteEncoderReversed = absoluteEncoderReversed;
        absoluteEncoder = new CANcoder(absoluteEncoderID); 



        driveMotor = new SparkMax(driveMotorID, MotorType.kBrushless);
        turningMotor = new SparkMax(turningMotorID, MotorType.kBrushless);

        //configure drive and turning motor of
        //the swerve module
        driveMotorConfig
        .inverted(driveMotorReversed);
        turnMotorConfig
        .inverted(turningMotorReversed);

        

        
        driveMotorConfig.encoder
        .positionConversionFactor(ModuleConstants.kDriveEncoderRot2Meter)
        .velocityConversionFactor(ModuleConstants.kDriveEncoderRPM2MeterPerSec);


        turnMotorConfig.encoder
        .positionConversionFactor(ModuleConstants.kTurningEncoderRot2Rad)
        .velocityConversionFactor(ModuleConstants.kTurningEncoderRPM2RadPerSec);

        
        turningPidController = new PIDController(ModuleConstants.kTurning, 0,0);

        turningPidController.enableContinuousInput(-Math.PI, Math.PI); // Tells PID that the system is circular
 
        resetEncoders();
        turningMotor.configure(turnMotorConfig,ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

        driveMotor.configure(driveMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    }

    public double getDrivePostion(){
        return driveMotor.getEncoder().getPosition();
    }

    public double getTurningPositon(){
        return  ((absoluteEncoder.getAbsolutePosition().getValueAsDouble() * 2 * Math.PI) - absoluteEncoderOffsetRad);
        //returns a values between - pi and pi
    }
    /*
     * returns drive velocity in meters per second
     */
    public double getDriveVelocity(){
        return driveMotor.getEncoder().getVelocity();
    }

    public double getTurningVelocity(){
        return turningMotor.getEncoder().getVelocity();
    }
    public double getAbsoluteEncoderRad(){

        double angle = (absoluteEncoder.getAbsolutePosition().getValueAsDouble());  // give the how much percent of a rotation were readin
        angle *= 2.0 * Math.PI; // convert to radians
        angle -= absoluteEncoderOffsetRad; 

        

        return angle * (absoluteEncoderReversed ? -1.0 : 1.0); // gives the Encoder value based on if the Encoder is reversed
    }

    public void resetEncoders(){
        driveMotor.getEncoder().setPosition(0);
        turningMotor.getEncoder().setPosition(getAbsoluteEncoderRad()); // reset the turning encoder to absoulute encoder value
    }
    /*gets the current velocity and rotation (in raidians) of the swerve module, also known as the swerve state 
     * 
     */
    public SwerveModuleState getState(){
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPositon()));
    }
    
    public SwerveModulePosition getSwerveModulePosition(){
        return new SwerveModulePosition(getDrivePostion(), new Rotation2d(getTurningPositon()));
    }

    /**
   *sets the desired speeds and rotation  of the swerve module  
   *
   * @param state current state of the module 
   */
    public void setDesiredState(SwerveModuleState state){


        if(Math.abs(state.speedMetersPerSecond) < 0.001) // if were barely moving do not reset the motors
        {
            stop();
            return;
        }
        /* 
        * Minimize the change in heading this swerve module state would require by potentially reversing
        * the direction the wheel spins. If this is used with the PIDController class's continuous input
        * functionality, the furthest a wheel will ever rotate is 90 degrees.
        */
        state.optimize(state.angle);
        
        
      
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPositon(), state.angle.getRadians()));
        SmartDashboard.putString("Swerve[" + absoluteEncoder.getDeviceID() + "] state", state.toString());



    }
    /*
 * stops driving and turning speeds
 */
    public void stop(){
        driveMotor.set(0);
        turningMotor.set(0);
        
    }

/*
 * send telemery on the swewrveModule 
 */
    public void sendToDashboard(){

        SmartDashboard.putNumber("Drive[" + absoluteEncoder.getDeviceID() + "] output", driveMotor.getAppliedOutput());


        SmartDashboard.putNumber("Turning[" + absoluteEncoder.getDeviceID() + "] output", turningMotor.getAppliedOutput());

        SmartDashboard.putNumber("DrivePos[" + absoluteEncoder.getDeviceID() + "]", getDrivePostion());



        SmartDashboard.putNumber("TurningPos[" + absoluteEncoder.getDeviceID() + "]", getTurningPositon());

        SmartDashboard.putNumber("AbsPos[" + absoluteEncoder.getDeviceID() + "] ", absoluteEncoder.getAbsolutePosition().getValueAsDouble());


    }


}