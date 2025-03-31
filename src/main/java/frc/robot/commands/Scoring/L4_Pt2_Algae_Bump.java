// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Scoring;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.AlgaePivotCommands.PIDAlgaePivot;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.commands.ElevatorCommands.ElevMotionMagicPID;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.CoralPivot;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class L4_Pt2_Algae_Bump extends SequentialCommandGroup {
  /** Creates a new Algae_Bump_After_Score. */
  public L4_Pt2_Algae_Bump(Elevator elevator, CoralPivot coralPivot, AlgaePivot algaePivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_FULL_RETRACT).withTimeout(0.7),

      new ParallelCommandGroup(
        new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_REEF_PICKUP).withTimeout(1.2),
        new ElevMotionMagicPID(elevator, Constants.Elevator.BOTTOM_HEIGHT).withTimeout(1.2)
    ),
      new PIDAlgaePivot(algaePivot, Constants.AlgaePivot.ENC_REVS_ELEVATOR_SAFE_POSITION).withTimeout(1)
    );
  }
}
