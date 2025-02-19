// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Targeting;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class ResetPoseWithLL extends Command {
  /** Creates a new ResetPoseWithLL. */

  private double pipeline = 0; 
  private double tv;
  private Swerve s_Swerve;    

  public ResetPoseWithLL(Swerve s_Swerve)  {
    // Use addRequirements() here to declare subsystem dependencies.
    this.s_Swerve = s_Swerve;
    addRequirements(s_Swerve);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
        // turn on the LED,  3 = force on
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(3);
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("pipeline").setNumber(pipeline);
   // s_Swerve.zeroHeading(); //added this to fix the targeting going the wrong way
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //get pose from LL, if valid target
    //get red/blue status from a station
    //if pose and blue, set odometry pose to LL pose
    //if pose and red, same but reverse the yaw
    //if no pose do nothing
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
