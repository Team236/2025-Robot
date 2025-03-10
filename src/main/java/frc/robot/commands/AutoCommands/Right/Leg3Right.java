// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Right;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDMakeAPSafeForElev;
import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralHoldCommands.CoralResetCount;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.Scoring.L2_Score;
import frc.robot.commands.Scoring.L4_Score;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetAllParallel;
import frc.robot.commands.Targeting.TargetAngle;
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
public class Leg3Right extends SequentialCommandGroup {
  /** Creates a new Leg3Right. */
  public Leg3Right(Swerve s_Swerve, Elevator elevator, AlgaePivot algeaPivot, CoralPivot coralPivot, CoralHold coralHold) {
    addCommands(
        Commands.parallel(
        //******NEED TO CHANGE TO "FALSE" BELOW????
        new DriveFwdAndSideAndTurn(s_Swerve, true ,125.5, -19, 6).withTimeout(3.5)//,
        //new PIDToElevSafePosition(algeaPivot).withTimeout(3.5)
        ),   
          //Use limelight to get exactly 10" from bumper to AprilTag
          new TargetAngle(s_Swerve).withTimeout(1),
          new TargetForwardDistance(s_Swerve, 10).withTimeout(1),
          new TargetSideDistance(s_Swerve, 0).withTimeout(1),
          //**** GET POSE WITH LIMELIGHT, BEFORE DRIVING WITH ODOMETRY
          new GetPoseWithLL(s_Swerve).withTimeout(0.25),
          //Needs to end  with coral scorer aligned with right branch of Reef
          new DriveFwdAndSideAndTurn(s_Swerve, false, 7, 6.5, 0),
          //**** RESET POSE TO VALUE FROM GetPoseWithLL
          new ResetPoseWithLL(s_Swerve).withTimeout(0.25),

          new L2_Score(elevator, coralHold, coralPivot, algeaPivot)
    );         
    
  }

}
