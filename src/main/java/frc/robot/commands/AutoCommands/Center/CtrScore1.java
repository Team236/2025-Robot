// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Center;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDToElevSafePosition;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.AutoCommands.EndDriveTrajectoryPID;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.commands.Scoring.L4_Score_AutoLeg1;
import frc.robot.commands.Targeting.GetPoseWithLL;
import frc.robot.commands.Targeting.ResetPoseWithLL;
import frc.robot.commands.Targeting.TargetSideDistance;
import frc.robot.subsystems.AlgaeHold;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;
import frc.robot.subsystems.Swerve;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class CtrScore1 extends SequentialCommandGroup {
  /** Creates a new FullRunParallel. */
  public CtrScore1(Swerve s_Swerve, Elevator elevator, AlgaePivot algaePivot, AlgaeHold algaeHold, CoralPivot coralPivot, CoralHold coralHold) {
    addCommands(

       new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT).withTimeout(0.5),

        new ParallelCommandGroup(        
          new DriveFwd(s_Swerve, false, Constants.AutoConstants.CENTER_FWD_DIST).withTimeout(2),
          new PIDToElevSafePosition(algaePivot).withTimeout(2),
          new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(2)  
        ), 

        new TargetSideDistance(s_Swerve, 0).withTimeout(1),
       // new FieldCentricTargetRight(s_Swerve).withTimeout(2),
        new GetPoseWithLL(s_Swerve).withTimeout(0.5),
        new DriveSideways(s_Swerve, false, -6.25).withTimeout(1.5),
        new ResetPoseWithLL(s_Swerve).withTimeout(0.5),
        //ADD COMMAND BELOW WHENEVER USING ELEVATOR PID AFTER DRIVE PID
         new EndDriveTrajectoryPID(s_Swerve).withTimeout(0.5),
         new L4_Score_AutoLeg1(elevator, coralHold, coralPivot, algaePivot),
         new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT).withTimeout(2),
        new DriveSideways(s_Swerve, false, -4).withTimeout(1.5),
         new EndDriveTrajectoryPID(s_Swerve).withTimeout(0.5)//,
         //new AlgaeL2Pickup(elevator, algaeHold, algaePivot).withTimeout(1),
         //new DriveReverse(s_Swerve, true, 12).withTimeout(2)
    );
  }
}


