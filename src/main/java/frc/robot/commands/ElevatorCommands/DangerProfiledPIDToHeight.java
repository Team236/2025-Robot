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

public class DangerProfiledPIDToHeight extends Command {
    private Elevator elevator;
  private double desiredHeight; //desired height in inches
  private double kP= Constants.Elevator.KP_ELEV;
  private double kI = Constants.Elevator.KI_ELEV;
  private double kD = Constants.Elevator.KD_ELEV;
  private final TrapezoidProfile.Constraints constraints;
  private final ProfiledPIDController pidController;

  //Must use meters for units in the profiledPIDController
  public DangerProfiledPIDToHeight(Elevator elevator, double desiredHeight) {
    this.elevator = elevator;
    this.desiredHeight = desiredHeight;
    addRequirements(elevator);

    //max velocity and acceleration in m/s 
    constraints = new TrapezoidProfile.Constraints(4, 4);
    pidController = new ProfiledPIDController(kP, kI, kD, constraints);
  }

  @Override
  public void initialize() {
    pidController.reset(0);
  }

  @Override
  public void execute() {
    pidController.setGoal(desiredHeight*0.0254);
    elevator.setElevSpeed(pidController.calculate(elevator.getElevatorHeightMeters()));
  }

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {
    return false;
  }
}
