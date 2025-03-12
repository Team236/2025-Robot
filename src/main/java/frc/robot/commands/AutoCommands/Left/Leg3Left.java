// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Left;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.AutoCommands.EndDriveTrajectoryPID;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.commands.Scoring.L4_Score;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetForwardDistance;
import frc.robot.commands.Targeting.TargetSideDistance;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Leg3Left extends SequentialCommandGroup {
  /** Creates a new Leg3Left. */
  public Leg3Left(Swerve s_Swerve, Elevator elevator, AlgaePivot algaePivot, CoralPivot coralPivot, CoralHold coralHold) {
 //MAKE ALL Y DISTANCES AND ALL ANGLES OPPOSITE TO Right
    addCommands(
        //******NEED TO CHANGE TO "FALSE" BELOW????\
        new DriveFwdAndSideAndTurn(s_Swerve, false ,125.5, 19, -6).withTimeout(3.5), //x 10
        //Commands.parallel(
        //new DriveFwdAndSideAndTurn(s_Swerve, true ,125.5, 19, -6).withTimeout(3.5), //x 106?
        //new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT)
        //),

        //new TargetSideDistance(s_Swerve, 0).withTimeout(1),
        //new TargetForwardDistance(s_Swerve, 0).withTimeout(1),
        //**** GET POSE WITH LIMELIGHT, BEFORE DRIVING WITH ODOMETRY
        new GetPoseWithLL(s_Swerve).withTimeout(0.5),
        //Needs to end  with coral scorer aligned with right branch of Reef
        //new DriveSideways(s_Swerve, false, -6.5).withTimeout(1.5), 
        //**** RESET POSE TO VALUE FROM GetPoseWithLL
        new ResetPoseWithLL(s_Swerve).withTimeout(0.5)//,
        // ,new EndDriveTrajectoryPID(s_Swerve).withTimeout(0.5),
        //new L4_Score(elevator, coralHold, coralPivot, algaePivot)
    );
  }
}
