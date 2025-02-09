package frc.robot.commands.AutoCommands.Right2Score;

import frc.robot.commands.AutoCommands.DriveReverse;
import frc.robot.commands.AutoCommands.DriveSideways;
import frc.robot.commands.AutoCommands.TurnOnly;
import frc.robot.subsystems.Swerve;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;


public class Leg3Series extends SequentialCommandGroup {

    public Leg3Series(Swerve s_Swerve) {
    addCommands(
    new DriveSideways(s_Swerve, false, 36).withTimeout(2),
    new DriveReverse(s_Swerve, true, -114.5).withTimeout(3),  
    new TurnOnly(s_Swerve, false, -6.6).withTimeout(1)
    );
  }
}
    