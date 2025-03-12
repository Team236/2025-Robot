// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Left;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveFwdAndTurn;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.Scoring.L2_Score;
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
public class Leg1Left extends SequentialCommandGroup {
  /** Creates a new Leg1Left. */
  public Leg1Left(Swerve s_Swerve, Elevator elevator, AlgaePivot algaePivot, CoralPivot coralPivot, CoralHold coralHold) {
     addCommands(
     //START ROBOT WITH BACK BUMPER FLUSH WITH BACK OF BLACK STARTING LINE, 91" from sideline
          new DriveFwdAndTurn(s_Swerve, false, 77.5,  61).withTimeout(1.5), //fwd 66.5
          new TargetSideDistance(s_Swerve, 0).withTimeout(1),
          new TargetForwardDistance(s_Swerve, 0).withTimeout(1),
          new GetPoseWithLL(s_Swerve).withTimeout(0.3),
          new DriveSideways(s_Swerve, false, 7.75).withTimeout(1.5),
          new ResetPoseWithLL(s_Swerve).withTimeout(0.25),
          new L4_Score(elevator, coralHold, coralPivot, algaePivot)

          //  new L4_Score_AutoLeg1(elevator, coralHold, coralPivot, algaePivot)
     );               

  }
}
