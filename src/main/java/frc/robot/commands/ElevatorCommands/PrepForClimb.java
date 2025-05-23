// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.AlgaePivot;
import frc.robot.subsystems.Elevator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepForClimb extends SequentialCommandGroup {
  /** Creates a new ClimbDownSequence. */
  public PrepForClimb(Elevator elevator, AlgaePivot algaePivot) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
     // new PIDToElevSafePosition(algaePivot).withTimeout(0.5),
      new ElevMotionMagicPID(elevator, Constants.Elevator.CLIMB_START_HEIGHT)
    );
  }
}
