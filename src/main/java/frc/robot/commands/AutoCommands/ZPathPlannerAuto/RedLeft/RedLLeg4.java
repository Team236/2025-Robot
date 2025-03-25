package frc.robot.commands.AutoCommands.ZPathPlannerAuto.RedLeft;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead 
   https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */

/* 
 *  alignment for this leg is LEFT from BLUE driverstation point of view 
 *  this start position assumes last leg drove away from 
 *  Reef E position as defined in PathPlanner application
*/
public class RedLLeg4 extends SequentialCommandGroup {

        public RedLLeg4(Swerve s_Swerve, boolean reversed) {

                TrajectoryConfig config = new TrajectoryConfig(
                                Constants.AutoConstants.kMaxSpeedMetersPerSecond,
                                Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                                .setKinematics(Constants.Swerve.swerveKinematics).setReversed(reversed);

                // All units in meters.
                Trajectory legTrajectory = TrajectoryGenerator.generateTrajectory(
                                // taken from Path: BlueR_leg4_toC_mirror_flip (RED-left)
                                new Pose2d(15.94018203751987, 0.6974960254372018, new Rotation2d(2.199114857512855)),
                                List.of(
                                    new Translation2d(15.717738050840083, 0.9686595273011758),
                                    //new Translation2d(15.46010421115841, 1.2733499805445234),
                                    new Translation2d(15.329810336005139, 1.4244764650051618),
                                    //new Translation2d(15.067196507707841, 1.7251236385699658),
                                    new Translation2d(14.935313594493195, 1.8750475887082994),
                                    //new Translation2d(14.671631569304129, 2.175241496245807),
                                    new Translation2d(14.540371004677507, 2.3260018496185744),
                                    new Translation2d(14.280414836361114, 2.6300716276811347) ),
                                new Pose2d(14.145958342323523, 2.7915053920063855, new Rotation2d(2.0943951023931957) ),
                                config);

                var thetaController = new ProfiledPIDController(
                                Constants.AutoConstants.kPThetaController, 0, 0,
                                Constants.AutoConstants.kThetaControllerConstraints);
                thetaController.enableContinuousInput(-Math.PI, Math.PI);

                SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                                legTrajectory,
                                s_Swerve::getPose,
                                Constants.Swerve.swerveKinematics,
                                new PIDController(Constants.AutoConstants.kPXController, 0, 0),
                                new PIDController(Constants.AutoConstants.kPYController, 0, 0),
                                thetaController,
                                s_Swerve::setModuleStates,
                                s_Swerve);

                addCommands(
                                new InstantCommand(() -> s_Swerve.setPose(legTrajectory.getInitialPose())),
                                swerveControllerCommand);
        }
}
