// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ElevatorCommands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.Elevator;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class DangerProfiledPIDToHeight extends Command {
    private Elevator elevator;
  private double desiredHeight; //desired height in inches
  private double kP= Constants.Elevator.KP_ELEV;
  private double kI = Constants.Elevator.KI_ELEV;
  private double kD = Constants.Elevator.KD_ELEV;
  private final TrapezoidProfile.Constraints constraints;
  private final ProfiledPIDController pidController;

  /** Creates a new TrapezoidPID. */

  //Must use meters for units in the profiledPIDController
  public DangerProfiledPIDToHeight(Elevator elevator, double desiredHeight) {
    this.elevator = elevator;
    this.desiredHeight = desiredHeight;
    addRequirements(elevator);

    //max velocity and acceleration in m/s 
    constraints = new TrapezoidProfile.Constraints(4, 4);
    pidController = new ProfiledPIDController(kP, kI, kD, constraints);
   // pidController.setSetpoint(desiredHeight);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    pidController.reset(0);

    //pidController.reset(desiredHeight*0.0254);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    pidController.setGoal(desiredHeight*0.0254);
    elevator.setElevSpeed(pidController.calculate(elevator.getElevatorHeightMeters()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //Removed statement below - may have been the cause of the jolting from one level to another!
  elevator.stopElevator();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
