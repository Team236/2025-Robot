package frc.robot.commands.CoralPivotCommands;


import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;

import frc.robot.subsystems.CoralPivot;


public class PIDCoralPivotWithWait extends SequentialCommandGroup {
  /** Creates a new L1_Score. */
  public PIDCoralPivotWithWait(CoralPivot coralPivot, double desiredRevs, double wait) {
    addCommands(
   // new PIDToElevSafePosition(algaePivot),
    new WaitCommand(wait),
    new PIDCoralPivot(coralPivot, desiredRevs).withTimeout(0.5)
   );
  }
}