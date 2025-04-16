package frc.robot.subsystems;

import frc.robot.SwerveModule;
import frc.robot.commands.Targeting.ResetFieldPoseWithTarget;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

import java.util.List;
import java.util.Optional;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;

public class Swerve extends SubsystemBase {
    public SwerveDriveOdometry swerveOdometry;
    public SwerveModule[] mSwerveMods;
    public Pigeon2 gyro;
    public SwerveDrivePoseEstimator m_poseEstimator; //replacement for swerve drive odometry; .update should be called every robot loop
    public double poseAngle;
    public double poseForwardDistance;
    public double poseSideDistance;

    //ll stuff
    private double pipeline = 0; 
    private double tv;
    public Pose2d poseLL; //want to use this pose after this command, after moving with odometry
    public Pose2d targetPose;

    //targeting
    public SwerveControllerCommand currentSwerveControllerCommand;
    public Trajectory currentTrajectory;

    public Swerve() {
        gyro = new Pigeon2(Constants.Swerve.pigeonID, "usb");
        //when calibrated on 3/31/25, gyro mount pose configs quarternion values were:
        //gyro.getConfigurator().apply(-0.041875,  0.012086,  0.005250, Z	-0.997314))
        gyro.getConfigurator().apply(new Pigeon2Configuration());
        gyro.setYaw(0);
        
        mSwerveMods = new SwerveModule[] {
            new SwerveModule(0, Constants.Swerve.Mod0.constants), //front left
            new SwerveModule(1, Constants.Swerve.Mod1.constants), //front right
            new SwerveModule(2, Constants.Swerve.Mod2.constants), //back left
            new SwerveModule(3, Constants.Swerve.Mod3.constants) //back right
        };

        swerveOdometry = new SwerveDriveOdometry(Constants.Swerve.swerveKinematics, getGyroYaw(), getModulePositions());

/* Here we use SwerveDrivePoseEstimator so that we can fuse odometry readings, for 3D targeting. 
The numbers used below are robot specific, and should be tuned. */
   m_poseEstimator = new SwerveDrivePoseEstimator(
     Constants.Swerve.swerveKinematics,
      gyro.getRotation2d(),
      new SwerveModulePosition[] {
        mSwerveMods[0].getPosition(), //front left
        mSwerveMods[1].getPosition(), //front right
        mSwerveMods[2].getPosition(), //back left
        mSwerveMods[3].getPosition()  //back right
      },
      new Pose2d(),
      VecBuilder.fill(0.05, 0.05, Math.toRadians(5)), //std deviations in X, Y (meters), and angle of the pose estimate
      VecBuilder.fill(0.5, 0.5, Math.toRadians(30)));  //std deviations  in X, Y (meters) and angle of the vision (LL) measurement
    }

//Methods start here:

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

    public void getLLPose() {
            // turn on the LED,  3 = force on
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
        NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
        // s_Swerve.zeroHeading(); //added this to fix the targeting going the wrong way

        //tv =1 means Limelight sees a target
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.isPresent()  && (tv == 1)) { //have alliance color and see target
            if (ally.get() == Alliance.Red){
            poseLL = LimelightHelpers.getBotPose2d_wpiRed("limelight");
            //  s_Swerve.setPose(poseLL); //do this later in ResetPose command
            }
            if (ally.get() == Alliance.Blue){
            poseLL = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
            // s_Swerve.setPose(poseLL); //do this later in ResetPose command
            }   
        }
        //else do nothing
    }

    public void getTargetPose(Pose2d targetPose) {
        Optional<Alliance> ally = DriverStation.getAlliance();
        if (ally.get() == Alliance.Blue){
            this.targetPose = new Pose2d(targetPose.getX(), targetPose.getY(), targetPose.getRotation().plus(new Rotation2d(Math.PI))); //do this later in ResetPose command
        }
        if (ally.get() == Alliance.Red){ // not sure why but we needed to swap these
            this.targetPose = targetPose; //do this later in ResetPose command
        }   

        // this.setPose(targetPose);

    }

    public void resetFldPoseWithTarget() {
        setPose(targetPose);
    }

    public void resetLLPose() {
        if (poseLL != null) {
            setPose(poseLL);
        }
    }

    /**
     * 
     * @param type "left" or "algae" or "right"
     * 
     */
    public void updateTargetingValues(String type) {
        Pose2d robotFieldPose;
        Pose2d targetFieldPose;
        double tv;
        int targetId;
        Optional<Alliance> alliance = DriverStation.getAlliance();
        
        tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //if target is seen
        targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); //target id
        
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);
        
        if (tv == 1 && Constants.Targeting.REEF_IDS.contains(targetId)) {
            SmartDashboard.putString("trajectory", "is not null (if)");
            robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");

            //april tag coordinates
            double x2 = Constants.Targeting.ID_TO_POSE.get(targetId).getX(); 
            double y2 = Constants.Targeting.ID_TO_POSE.get(targetId).getY(); 
            double angle2 = Constants.Targeting.ID_TO_POSE.get(targetId).getRotation().getRadians();

            //Get the AprilTag pose now, then reset the robot's pose to this value at the end of MetricDriveFwdAndSideAndTurn
            //(after targeting) so that the driving is field oriented after targeting:
            this.getTargetPose(new Pose2d(x2, y2, new Rotation2d(angle2)));
            
            //robotFieldPose is from center of robot
            double angle1 = robotFieldPose.getRotation().getRadians();
            double x1 = robotFieldPose.getX() - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.cos(angle2) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.sin((angle2));
            double y1 = robotFieldPose.getY() + (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.cos((angle2)) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.sin((angle2));

            if (type.equals("right")) {
                y1 -= Constants.Targeting.DIST_TAG_RIGHT_BRANCH * Math.cos((angle2)) * 0.0254;
                x1 += Constants.Targeting.DIST_TAG_RIGHT_BRANCH * Math.sin((angle2)) * 0.0254;
            } else if (type.equals("algae")) {
                y1 -= Constants.Targeting.DIST_ALGAE_CENTERED_LL * Math.cos((angle2)) * 0.0254;
                x1 += Constants.Targeting.DIST_ALGAE_CENTERED_LL * Math.sin((angle2)) * 0.0254;
            } else if (type.equals("left")) {
                y1 += Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.cos((angle2)) * 0.0254;
                x1 -= Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.sin((angle2)) * 0.0254;
            }
      
           // SmartDashboard.putNumber("Target ID", targetId);
           // SmartDashboard.putNumber("x1: ", x1 / 0.0254);
           // SmartDashboard.putNumber("y1: ", y1/ 0.0254);
           // SmartDashboard.putNumber("angle1", Units.radiansToDegrees(angle1));
           // SmartDashboard.putNumber("x2: ", x2/ 0.0254);
           // SmartDashboard.putNumber("y2: ", y2/ 0.0254);
           // SmartDashboard.putNumber("angle2", Units.radiansToDegrees(angle2));
            
            // DRIVE SEGMENT
            double deltaFwd = x2 - x1;
            double deltaSide = y2 - y1;
            double deltaAngle = angle2- angle1;

            boolean reversed = false; 

            TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

            // An example trajectory to follow.  All units in meters.
            Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
                new Pose2d(x1, y1, new Rotation2d((angle1))),
            // Pass through these interior waypoints
            List.of(
                    new Translation2d((x1+0.05*deltaFwd), (y1+0.05*deltaSide)), 
                    new Translation2d((x1+0.1*deltaFwd), (y1+0.1*deltaSide)),
                    new Translation2d((x1+0.15*deltaFwd), (y1+0.15*deltaSide)),
                    new Translation2d((x1+0.2*deltaFwd), (y1+0.2*deltaSide)), 
                    new Translation2d((x1+0.25*deltaFwd), (y1+0.25*deltaSide)),
                    new Translation2d((x1+0.3*deltaFwd), (y1+0.3*deltaSide)),
                    new Translation2d((x1+0.35*deltaFwd), (y1+0.35*deltaSide)), 
                    new Translation2d((x1+0.4*deltaFwd), (y1+0.4*deltaSide)),
                    new Translation2d((x1+0.45*deltaFwd), (y1+0.45*deltaSide)), 
                    new Translation2d((x1+0.5*deltaFwd), (y1+0.5*deltaSide)),
                    new Translation2d((x1+0.55*deltaFwd), (y1+0.55*deltaSide)),
                    new Translation2d((x1+0.6*deltaFwd), (y1+0.6*deltaSide)), 
                    new Translation2d((x1+0.65*deltaFwd), (y1+0.65*deltaSide)),
                    new Translation2d((x1+0.7*deltaFwd), (y1+0.7*deltaSide)),
                    new Translation2d((x1+0.75*deltaFwd), (y1+0.75*deltaSide)), 
                    new Translation2d((x1+0.8*deltaFwd), (y1+0.8*deltaSide)),
                    new Translation2d((x1+0.85*deltaFwd), (y1+0.85*deltaSide)), 
                    new Translation2d((x1+0.9*deltaFwd), (y1+0.9*deltaSide)),
                    new Translation2d((x1+0.95*deltaFwd), (y1+0.95*deltaSide))
                    ),  
            // End here
            new Pose2d(x2, y2, new Rotation2d((angle2 + Math.PI))), //add 180 because target and robot are facing opposite directions
            config);

            currentTrajectory = exampleTrajectory;
                

            SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                    exampleTrajectory,
                    this::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    this::setModuleStates,
                    this);
            
            currentSwerveControllerCommand = swerveControllerCommand;
        } else {
            SmartDashboard.putString("trajectory", "is null (else)");

            boolean reversed = false;
            TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

            // An example trajectory to follow.  All units in meters.
            Trajectory exampleTrajectory =
            TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
            new Pose2d(0, 0, new Rotation2d(0)),
            // Pass through these interior waypoints
            List.of(
                    new Translation2d(0.01, 0.01)
                    ),  
            // End here
            new Pose2d(0.02, 0.02, new Rotation2d(0)), //add 180 because target and robot are facing opposite directions
            config);
            currentTrajectory = exampleTrajectory; 

            SwerveControllerCommand swerveControllerCommand =
                new SwerveControllerCommand(
                    exampleTrajectory,
                    this::getPose,
                    Constants.Swerve.swerveKinematics,
                    new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                    new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                    thetaController,
                    this::setModuleStates,
                    this);
            currentSwerveControllerCommand = swerveControllerCommand;
        }
    }
    
    public void setDefaultValues() {  
        boolean reversed = false; 
        Trajectory exampleTrajectory;

        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        //This trajecotry is just used to avoid nulls
        //sets the beginning pose, waypoints and end pose all to the value of the current pose (getPose())
        //Used 2 waypoints since that may be the minimum number 
        exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            getPose(),
            List.of(
              getPose().getTranslation(),
              getPose().getTranslation()
                   ),  
            getPose(),
            config);
            currentTrajectory = exampleTrajectory;
            
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        
        //This command is just used to avoid nulls
        // But will get called if no target seen, so kpX=kpY=0 so no movement if no target
        SwerveControllerCommand swerveControllerCommand =
        new SwerveControllerCommand(
            exampleTrajectory,
            this::getPose,
            Constants.Swerve.swerveKinematics,
            new PIDController(0, 0, 0),
            new PIDController(0, 0, 0),
            thetaController,
            this::setModuleStates,
            this);
            currentSwerveControllerCommand = swerveControllerCommand;
    }

    public void setupValues() {
    Pose2d robotFieldPose;
    Pose2d targetFieldPose;
    double tv;
    int targetId;
    Optional<Alliance> alliance = DriverStation.getAlliance();
    Trajectory exampleTrajectory;
  
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0); //if target is seen
    targetId = (int) NetworkTableInstance.getDefault().getTable("limelight").getEntry("tid").getDouble(0); //target id

    SmartDashboard.putNumber("TV: ", tv);

    if (tv == 1 && Constants.Targeting.REEF_IDS.contains(targetId)) {
   
      robotFieldPose = LimelightHelpers.getBotPose2d_wpiBlue("limelight");
 
    //  SmartDashboard.putNumber("x1 robot center: ", robotFieldPose.getX() / 0.0254);
    //  SmartDashboard.putNumber("y1 robot center: ", robotFieldPose.getY()/ 0.0254);
      
      //april tag coordinates
      double x2 = Constants.Targeting.ID_TO_POSE.get(targetId).getX(); //*Math.sin((angle2));
      double y2 = Constants.Targeting.ID_TO_POSE.get(targetId).getY(); //*Math.cos((angle2));
      double angle2 = Constants.Targeting.ID_TO_POSE.get(targetId).getRotation().getRadians();

      //Get the AprilTag pose now, then reset the pose to this value at the end of MetricDriveFwdAndSideAndTurn
      //(after targeting) so that the driving is field oriented after targeting:
      getTargetPose(new Pose2d(x2, y2, new Rotation2d(angle2)));
      
      //robotFieldPose is from center of robot
      double angle1 = robotFieldPose.getRotation().getRadians();
      double x1 = robotFieldPose.getX() - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.cos(angle2) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.sin((angle2));
      double y1 = robotFieldPose.getY() + (Constants.Targeting.DIST_ROBOT_CENTER_TO_LL_SIDEWAYS*(0.0254))*Math.cos((angle2)) - (Constants.Targeting.DIST_ROBOT_CENTER_TO_FRONT_WITH_BUMPER*(0.0254)) * Math.sin((angle2));

      y1 += Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.cos((angle2)) * 0.0254;
      x1 -= Constants.Targeting.DIST_TAG_LEFT_BRANCH * Math.sin((angle2)) * 0.0254;
      
     SmartDashboard.putNumber("Target IDDD", targetId);
      SmartDashboard.putNumber("x1: ", x1 / 0.0254);
      SmartDashboard.putNumber("y1: ", y1/ 0.0254);
      SmartDashboard.putNumber("angle1", Units.radiansToDegrees(angle1));
      SmartDashboard.putNumber("x2: ", x2/ 0.0254);
      SmartDashboard.putNumber("y2: ", y2/ 0.0254);
      SmartDashboard.putNumber("angle2", Units.radiansToDegrees(angle2));
      

      // DRIVE SEGMENT
 
        double deltaFwd = x2 - x1;
        double deltaSide = y2 - y1;
        double deltaAngle = angle2- angle1;

        boolean reversed = false; 

        TrajectoryConfig config =
            new TrajectoryConfig(
                    Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                    Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

        // An example trajectory to follow.  All units in meters.
        exampleTrajectory =
        TrajectoryGenerator.generateTrajectory(
            // Start at the origin facing the +X direction
             new Pose2d(x1, y1, new Rotation2d((angle1))),
            // Pass through these interior waypoints
            List.of(
                   new Translation2d((x1+0.05*deltaFwd), (y1+0.05*deltaSide)), 
                   new Translation2d((x1+0.1*deltaFwd), (y1+0.1*deltaSide)),
                   new Translation2d((x1+0.15*deltaFwd), (y1+0.15*deltaSide)),
                   new Translation2d((x1+0.2*deltaFwd), (y1+0.2*deltaSide)), 
                   new Translation2d((x1+0.25*deltaFwd), (y1+0.25*deltaSide)),
                   new Translation2d((x1+0.3*deltaFwd), (y1+0.3*deltaSide)),
                   new Translation2d((x1+0.35*deltaFwd), (y1+0.35*deltaSide)), 
                   new Translation2d((x1+0.4*deltaFwd), (y1+0.4*deltaSide)),
                   new Translation2d((x1+0.45*deltaFwd), (y1+0.45*deltaSide)), 
                   new Translation2d((x1+0.5*deltaFwd), (y1+0.5*deltaSide)),
                   new Translation2d((x1+0.55*deltaFwd), (y1+0.55*deltaSide)),
                   new Translation2d((x1+0.6*deltaFwd), (y1+0.6*deltaSide)), 
                   new Translation2d((x1+0.65*deltaFwd), (y1+0.65*deltaSide)),
                   new Translation2d((x1+0.7*deltaFwd), (y1+0.7*deltaSide)),
                   new Translation2d((x1+0.75*deltaFwd), (y1+0.75*deltaSide)), 
                   new Translation2d((x1+0.8*deltaFwd), (y1+0.8*deltaSide)),
                   new Translation2d((x1+0.85*deltaFwd), (y1+0.85*deltaSide)), 
                   new Translation2d((x1+0.9*deltaFwd), (y1+0.9*deltaSide)),
                   new Translation2d((x1+0.95*deltaFwd), (y1+0.95*deltaSide))
                   ),  
            // End here
            new Pose2d(x2, y2, new Rotation2d((angle2 + Math.PI))), //add 180 because target and robot are facing opposite directions
            config);     
        currentTrajectory = exampleTrajectory;
            
        var thetaController =
            new ProfiledPIDController(
                Constants.AutoConstants.kPThetaController, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand =
            new SwerveControllerCommand(
                exampleTrajectory,
                this::getPose,
                Constants.Swerve.swerveKinematics,
                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                thetaController,
                this::setModuleStates,
                this);
            currentSwerveControllerCommand = swerveControllerCommand;

         }
    }

/** Updates the field relative position of the robot. */
//THIS METHOD WAS NOT USED IN 2025 SEASON - FIND OUT HOW TO IMPLEMENT IT
//USED IN AUTONOMOUS PERIODIC IN LL EXAMPLE CODE swerve-megatag-odometry"
    public void MegaTag2UpdateOdometry() {
        /* Replaced below with m_poseEstimator.update(getGyroYaw(), getModulePositions()); as done in periodic for swerve odometry
        m_poseEstimator.update(
            gyro.getRotation2d(),
            new SwerveModulePosition[] {
                mSwerveMods[0].getPosition(), //front left
                mSwerveMods[1].getPosition(), //front right
                mSwerveMods[2].getPosition(), //back left
                mSwerveMods[3].getPosition()  //back right
            });
        */   m_poseEstimator.update(getGyroYaw(), getModulePositions());

        boolean useMegaTag2 = true; //set to false to use MegaTag1
        boolean doRejectUpdate = false;
        // evaluating which Megatag one or two to use based on above boolean value and 
        // only incorporate Limelight's estimates when more than one tag is visible (tagcount >= 1)
        if(useMegaTag2 == false)
        {
          LimelightHelpers.PoseEstimate mt1 = LimelightHelpers.getBotPoseEstimate_wpiBlue("limelight");
          if(mt1.tagCount == 1 && mt1.rawFiducials.length == 1)
          {
            if(mt1.rawFiducials[0].ambiguity > .7) { doRejectUpdate = true; }
            if(mt1.rawFiducials[0].distToCamera > 3) { doRejectUpdate = true; }
          }
          if(mt1.tagCount == 0) { doRejectUpdate = true; }
          if(!doRejectUpdate) {     // if doRejectUpdate is false (or NOT true), then update the pose estimator
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.5,.5,9999999));
            m_poseEstimator.addVisionMeasurement(
                mt1.pose,
                mt1.timestampSeconds);
          }
        }
        else if (useMegaTag2 == true)
        {   // only incorporate Limelight's estimates when more than one tag is visible (tagcount >= 1)
          LimelightHelpers.SetRobotOrientation("limelight", m_poseEstimator.getEstimatedPosition().getRotation().getDegrees(), 0, 0, 0, 0, 0);
          LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
          if(Math.abs(gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore vision updates
          {
            doRejectUpdate = true;
          }
          if(mt2.tagCount == 0)
          {
            doRejectUpdate = true;
          }
          if(!doRejectUpdate)   // if doRejectUpdate is false (or NOT true), then update the pose estimator
          {
            m_poseEstimator.setVisionMeasurementStdDevs(VecBuilder.fill(.7,.7,9999999));
            m_poseEstimator.addVisionMeasurement(
                mt2.pose,
                mt2.timestampSeconds);
          }
      }
    }

    @Override
    public void periodic(){
       swerveOdometry.update(getGyroYaw(), getModulePositions());
      // swerveOdometry.getPoseMeters();

      //TO ADD IN VISION MEASUREMENTS TO ODOMETRY, REPLACE LINE ABOVE WITH THE LINE BELOW (???)
      //TEST BY USING FIELD CENTRIC TARGETING AND THEN BY WATCHING THE getPoseMeters ON DASHBOARD WHILE DRIVING
      //MegaTag2UpdateOdometry(); //method in Swerve subysystem
      //swerveOdometry.getPoseMeters();



   //  SmartDashboard.putNumber("limelight standoff fwd", LimelightHelpers.getTargetPose_CameraSpace("limelight")[2]);

        //for(SwerveModule mod : mSwerveMods){
         // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " CANcoder degrees", mod.getCANcoder().getDegrees());
          //SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Angle degrees", mod.getPosition().angle.getDegrees());
          // SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity", mod.getState().speedMetersPerSecond);
          //Can't use m/s in the key!! SmartDashboard.putNumber("Mod " + mod.moduleNumber + " Velocity m/s", mod.getState().speedMetersPerSecond);
         //}
    
         /* 
       poseAngle = LimelightHelpers.getTargetPose_CameraSpace("limelight")[5];
       SmartDashboard.putNumber("TargetingAngle in swerve: ", poseAngle);
       poseForwardDistance = LimelightHelpers.getTargetPose_ameraSpace("limelight")[2];
      SmartDashboard.putNumber("TargetingForwardDistance in swerve: ", poseForwardDistance / 0.0254);
      poseSideDistance = LimelightHelpers.getTargetPose_CameraSpace("limelight")[0];
       SmartDashboard.putNumber("TargetingSideDistance in swerve: ", poseSideDistance / 0.0254);
       */
     }

}       

