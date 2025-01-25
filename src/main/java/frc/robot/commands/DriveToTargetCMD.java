package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
import frc.robot.config.LimelightHelpers;
import frc.robot.subsystems.SwerveSub;

public class DriveToTargetCMD extends Command {
     private final SwerveSub swerveSubsystem;

     // naming system : nameOfVarible_referenceFrame_Units
     private Translation2d initialEncoderDisplacmentFromStartPose_RobotSpace_Meters;
     private Translation2d currentEncoderDisplacmentFromStartPose_RobotSpace_Meters;
     private Translation2d changeInEncoderDisplacmentFromStartPose_Translation_Meters;


     private Translation2d initialRobotDisplacementFromTarget_TargetSpace_Meters;
     private Translation2d currentRobotDisplacementFromTarget_TargetSpace_Meters;

    private boolean initialScan;
    
     public DriveToTargetCMD(
        SwerveSub swerveSubsystem 
    ){
        this.swerveSubsystem = swerveSubsystem;
        addRequirements(swerveSubsystem);
    }


    @Override
    public void initialize(){
            //checks for april tag
            initialScan = LimelightHelpers.getTV("limelight");
            if(!initialScan){
                return;
            }

            /* finds displacement from the target 
    
            TargetSpace reference
            3d Cartesian Coordinate System with (0,0,0) at the center of the target.

            X+ → Pointing to the right of the target (If you are looking at the target)
            
            Y+ → Pointing downward
            
            Z+ → Pointing out of the target (orthogonal to target's plane).
            */
            

            Pose3d targetPose_Meters = LimelightHelpers.getBotPose3d_TargetSpace("limelight");

            initialRobotDisplacementFromTarget_TargetSpace_Meters = new Translation2d(
                targetPose_Meters.getX(),
                targetPose_Meters.getZ());
                

            /*  RobotSpace reference
            3d Cartesian Coordinate System with (0,0,0) located at the center of the robot’s frame projected down to the floor.

            X+ → Pointing forward (Forward Vector)
            
            Y+ → Pointing toward the robot’s right (Right Vector)
            
            Z+ → Pointing upward (Up Vector)
            */
            //records current Pose

            Pose2d currentRobotPose_meters = swerveSubsystem.getPose();
            initialEncoderDisplacmentFromStartPose_RobotSpace_Meters = new Translation2d(
                currentRobotPose_meters.getX(),
             currentRobotPose_meters.getY());

            

            
            
    }
    @Override
    public void execute(){
        // find the encoder displacment from the april tag localization
        //t
        Pose2d currentRobotPose_meters = swerveSubsystem.getPose();
        currentEncoderDisplacmentFromStartPose_RobotSpace_Meters = new Translation2d(
            currentRobotPose_meters.getX(),
         currentRobotPose_meters.getY());
         //find the change robot displace since our initial encoder displament
         // delta_d = d_2 - d_1
         changeInEncoderDisplacmentFromStartPose_Translation_Meters =
         currentEncoderDisplacmentFromStartPose_RobotSpace_Meters.minus
         (initialEncoderDisplacmentFromStartPose_RobotSpace_Meters);

         //find the change inrobot  displacement from target 
         // T = t - deleta_d
        currentRobotDisplacementFromTarget_TargetSpace_Meters = 
        initialRobotDisplacementFromTarget_TargetSpace_Meters.
        plus(changeInEncoderDisplacmentFromStartPose_Translation_Meters);

        

        //drive robot to position based
        double[] powerOutput = swerveSubsystem.driveToTarget(
            currentRobotDisplacementFromTarget_TargetSpace_Meters.getX(),
            currentRobotDisplacementFromTarget_TargetSpace_Meters.getY()); 

        ChassisSpeeds chassisSpeeds;


            chassisSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
            powerOutput[0],
           -powerOutput[1], 
           swerveSubsystem.turnParrelleToCoralStationPower(),
           swerveSubsystem.getRotation2d());

            // convert chassis speeds to individual module states; later to switch to velocity
             SwerveModuleState[] moduleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(chassisSpeeds);
            // set state to each wheel
            swerveSubsystem.setModuleStates(moduleStates);
            
            sendTelemetry();

        



    }

    @Override 
    public boolean isFinished(){
        //if no april tag is detect on initial scan, end command
        if(!initialScan){
            return true;
        }
        double[] PositionControllerError_Meters = swerveSubsystem.getPositionControllerError_Meters();
        //when the position x position error is below the threshold end command
        if(Math.abs(PositionControllerError_Meters[0]) <= Constants.DriveConstants.PositionControllers.drive.tolerance){
            return true;
        }
        //when the position y position error is below the threshold end command
        if(Math.abs(PositionControllerError_Meters[1]) <= Constants.DriveConstants.PositionControllers.drive.tolerance){
            return true;
        }        //when the position theta position error is below the threshold end command
        if(Math.abs(swerveSubsystem.getHeadingError_degrees()) <= Constants.DriveConstants.PositionControllers.turning.tolerance){
            return true;
        }
        return false;
    }
    @Override 
    public void end(boolean interrupted){
        swerveSubsystem.stopModules();
    }
    public void sendTelemetry(){
        SmartDashboard.putNumber("initialEncoderDisplacmentFromStartPose_x", initialEncoderDisplacmentFromStartPose_RobotSpace_Meters.getX());
        SmartDashboard.putNumber("initialEncoderDisplacmentFromStartPose_y", initialEncoderDisplacmentFromStartPose_RobotSpace_Meters.getY());

        SmartDashboard.putNumber("currentEncoderDisplacmentFromStartPose_x", currentEncoderDisplacmentFromStartPose_RobotSpace_Meters.getX());
        SmartDashboard.putNumber("currentEncoderDisplacmentFromStartPose_y", currentEncoderDisplacmentFromStartPose_RobotSpace_Meters.getY());

        SmartDashboard.putNumber("changeInEncoderDisplacmentFromStartPose_x", changeInEncoderDisplacmentFromStartPose_Translation_Meters.getX());
        SmartDashboard.putNumber("changeInEncoderDisplacmentFromStartPose_y", changeInEncoderDisplacmentFromStartPose_Translation_Meters.getY());


        SmartDashboard.putNumber("initialRobotDisplacementFromTarget_x", initialRobotDisplacementFromTarget_TargetSpace_Meters.getX());
        SmartDashboard.putNumber("initialRobotDisplacementFromTarget_x", initialRobotDisplacementFromTarget_TargetSpace_Meters.getY());

        SmartDashboard.putNumber("currentRobotDisplacementFromTarget_y", currentRobotDisplacementFromTarget_TargetSpace_Meters.getY());
        SmartDashboard.putNumber("currentRobotDisplacementFromTarget_x", currentRobotDisplacementFromTarget_TargetSpace_Meters.getX());
    }

}
