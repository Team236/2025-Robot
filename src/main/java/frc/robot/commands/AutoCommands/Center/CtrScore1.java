// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.AutoCommands.Center;

import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDMakeAPSafeForElev;
import frc.robot.commands.AutoCommands.DriveFwd;
import frc.robot.commands.AutoCommands.DriveFwdAndSideAndTurn;
import frc.robot.commands.AutoCommands.DriveReverse;
import frc.robot.commands.CoralHoldCommands.CoralRelease;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.DangerPIDToHeight;
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
  public CtrScore1(Swerve s_Swerve, Elevator elevator, AlgaePivot algeaPivot, CoralPivot coralPivot, CoralHold coralHold) {
    addCommands(
      //TODO:  add the commands for scoring 
      Commands.parallel(
        new DriveFwd(s_Swerve, false, Constants.AutoConstants.CENTER_FWD_DIST),
        new PIDMakeAPSafeForElev(algeaPivot)
        ),
      Commands.parallel(
        new DangerPIDToHeight(elevator, Constants.Elevator.L4_HEIGHT),
        new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LEVEL4)
        ),
       new CoralRelease(coralHold, Constants.CoralHold.L4_RELEASE_SPEED).withTimeout(2),
       new PIDMakeAPSafeForElev(algeaPivot),
       new DangerPIDToHeight(elevator, Constants.Elevator.TELEOP_HEIGHT)
    );

  }
}