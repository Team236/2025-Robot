// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.CoralHoldCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.CoralHold;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class CoralGrabWithCounter extends Command {
  /** Creates a new CoralGrabWithCounter. */
  //test comment
  private CoralHold coralHold;
  private double speed;

  public CoralGrabWithCounter(CoralHold coralHold, double speed) {
    this.speed = speed;
    this.coralHold = coralHold;
    addRequirements(this.coralHold);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //coralHold.resetCount(); //DELETE THIS IF USING THE LIGHT SENSOR AS TRUE/FALSE, NOT COUNTING
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    coralHold.setCoralHSpeed(speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
     coralHold.coralHStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return coralHold.getLightSensorState();
    //return coralHold.getCoralHCount() > 0;
  }
}