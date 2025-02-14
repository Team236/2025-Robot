package frc.robot.subsystems;

import java.util.List;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.FollowPathCommand;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.util.DriveFeedforwards;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.SwerveModule;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator m_poseEstimator;
    public double poseAngle;
    public double poseForwardDistance;
    public double poseSideDistance;

         // pathPlanner stuff
    public static PathPlannerPath pathPlannerPath;
    private static RobotConfig robotConfig;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "usb");
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);

        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants), //front left
            new SwerveModule(1, Constants.Swerve.Mod1.constants), //front right
            new SwerveModule(2, Constants.Swerve.Mod2.constants), //back left
            new SwerveModule(3, Constants.Swerve.Mod3.constants) //back right
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

        // read the robot configuration from the PathPlanner GUI settings
        try {
            robotConfig = RobotConfig.fromGUISettings();
        } catch (Exception  e) {
            DriverStation.reportError("ParseException" + e.getMessage(), e.getStackTrace());
        }

/* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings, for 3D targeting. 
The numbers used below are robot specific, and should be tuned. */
   m_poseEstimator = new SwerveDrivePoseEstimator(
     Constants.Swerve.swerveKinematics,
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
        mSwerveMods[0].getPosition(),       //front left
        mSwerveMods[1].getPosition(),       //front right
        mSwerveMods[2].getPosition(),       //back left
        mSwerveMods[3].getPosition()  },    //back right   
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Math.toRadians(5)), //std deviations in X, Y (meters), and angle of the pose estimate
      VecBuilder.fill(0.5, 0.5, Math.toRadians(30))); //std deviations in X, Y (meters) and angle of the vision (LL) measurement
    }

//Methods start here:
    public void drive(ChassisSpeeds chassisSpeed , DriveFeedforwards driveFeedforward){
        
    }

    public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        SwerveModuleState[] swerveModuleStates =
            Constants.Swerve.swerveKinematics.toSwerveModuleStates(
                fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation, 
                                    getHeading()
                                )
                                : new ChassisSpeeds(
                                    translation.getX(), 
                                    translation.getY(), 
                                    rotation)
                                );
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.Swerve.maxSpeed);

        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
        }
    }    

    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.Swerve.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.moduleNumber], false); //closed loop auto
        }
    }

    public SwerveModuleState[] getModuleStates(){
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveModule mod : mSwerveMods){
            states[mod.moduleNumber] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions(){
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveModule mod : mSwerveMods){
            positions[mod.moduleNumber] = mod.getPosition();
        }
        return positions;
    }

    public Pose2d getPose() {
        return swerveOdometry.getPoseMeters();
    }

    public void setPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public void resetPose(Pose2d pose) {
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), pose);
    }

    public Rotation2d getHeading(){
        return getPose().getRotation();
    }

    public void setHeading(Rotation2d heading){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), heading));
    }

    public void zeroHeading(){
        swerveOdometry.resetPosition(getGyroYaw(), getModulePositions(), new Pose2d(getPose().getTranslation(), new Rotation2d()));
    }

    public Rotation2d getGyroYaw() {
        //return Rotation2d.fromDegrees(gyro.getYaw().getValue());
       return Rotation2d.fromDegrees(gyro.getYaw().getValueAsDouble());
    }

    public void resetModulesToAbsolute(){
        for(SwerveModule mod : mSwerveMods){
            mod.resetToAbsolute();
        }
    }

    
    public ChassisSpeeds getSpeeds() {
        return Constants.Swerve.swerveKinematics.toChassisSpeeds(getModuleStates());
    }
// **********

    // public ChassisSpeeds getCurrentSpeeds() {
    //     // return Constants.Swerve.swerveKinematics.toChassisSpeeds(
    //     //     Constants.Swerve.swerveKinematics.toSwerveModuleStates(gyro.);
    //     // );
    // }
    // public ChassisSpeeds getRobotRelativeSpeeds()
    // {
    //     return ChassisSpeeds.fromFieldRelativeSpeeds(
    //         // swerveOdometry
    //         // RobotContainer.driverController.getLeftX(),
    //         // RobotContainer.driverController.getLeftY(),
    //         // RobotContainer.  getRightX(),
    //         // getHeading()
    //         return new ChassisSpeeds(
    //             gyro.getAngularVelocityXDevice(),
    //             gyro.getAngularVelocityYDevice(),
    //             gyro.getA(),
    //         )
    //     );
    // }
/* 
    public double getLLAngleDegrees() {
        return (LimelightHelpers.getTargetPose_CameraSpace("limelight")[5]);
    }

    public double getLLFwdDistMeters() {
        return (LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]);
    }

    public double getLLSideDistMeters() {
        return (LimelightHelpers.getTargetPose_CameraSpace("limelight")[0]);
    }

    public double getLLFwdDistInch() {
        return Units.metersToInches(LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]);
    }
    
    public double getLLSideDistInch() {
        return Units.metersToInches(LimelightHelpers.getTargetPose_CameraSpace("limelight")[0]);
    }
 */

    @Override
    public void periodic(){
        //SmartDashboard.putNumber("limelight standoff fwd", LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]);

       swerveOdometry.update(getGyroYaw(), getModulePositions());

         for(SwerveModule mod : mSwerveMods){
           SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
           SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder degrees", mod.getCANcoder().getDegrees());
           SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle degrees", mod.getPosition().angle.getDegrees());
           SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
          //Can't use m/s in the key!! SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity m/s", mod.getState().speedMetersPerSecond);
         }
         /* 
       poseAngle = LimelightHelpers.getTargetPose_CameraSpace("limelight")[5];
       SmartDashboard.putNumber("TargetingAngle in swerve: ", poseAngle);
       poseForwardDistance = LimelightHelpers.getTargetPose_CameraSpace("limelight")[2];
        SmartDashboard.putNumber("TargetingForwardDistance in swerve: ", poseForwardDistance / 0.0254);
        poseSideDistance = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0];
       SmartDashboard.putNumber("TargetingSideDistance in swerve: ", poseSideDistance / 0.0254);
       */
    }


    // PathPlanner method to follow path specified in the calling of the method from a command class
    public Command followPathCommand(String pathName) {
        pathPlannerPath = PathPlannerPath.fromPathFile(pathName);
        
        
        Command m_pathCommand;
        //List<Pose2d> poseArray = pathPlannerPath.getPathPoses();
        try { 

        // Create a path following command using AutoBuilder. This will also trigger event markers.
        try {
                m_pathCommand = AutoBuilder.followPath(pathPlannerPath);
            } catch (Exception e) {System.out.print("something bad: " + e.getStackTrace() )  
        }
        return m_pathCommand;
    }
}

