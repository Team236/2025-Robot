// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralHoldCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.CoralPivotCommands.PIDCoralPivot;
import frc.robot.subsystems.CoralHold;
import frc.robot.subsystems.CoralPivot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class CoralSeqRelease extends ParallelCommandGroup {
  /** Creates a new CoralSeqRelease. */
  public CoralSeqRelease(CoralPivot coralPivot, CoralHold coralHold) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new PIDCoralPivot(coralPivot, Constants.CoralPivot.ENC_REVS_LOADING),
      new CoralGrab(coralHold, Constants.CoralHold.L2_RELEASE_SPEED)
    );
  }
}
