// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Pathautos;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class SinglePathFollowing extends Command {
private Swerve swerve;
private String pathName="Reef-K_Coral-10";
// private String hardPath ="Reef-K_Coral-10";
private PathPlannerPath path;

  /** Creates a new SinglePathFollowing. */
  public SinglePathFollowing(Swerve swerve, String pathName) {
    this.swerve = swerve;
    this.pathName = pathName;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
  }

   /** Creates a new SinglePathFollowing. */
   public SinglePathFollowing(Swerve swerve) {
    this.swerve = swerve;
    
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(this.swerve);
  }


// Called when the command is initially scheduled.
  @Override
  public void initialize() {
    try{
      // PathPlannerAuto.getPathGroupFromAutoFile("auto1");
      // PathPlannerPath path = PathPlannerPath.fromPathFile("Reef-K_Coral-10.path");
      this.path = PathPlannerPath.fromPathFile(pathName);

    } catch (Exception e) { 
      System.out.println("report: "+ e.getStackTrace() );
    }
    
    

    AutoBuilder.followPath(this.path);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //TODO command the this.swerve to drive the pathPlannerPath
    // swerve.drive();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
