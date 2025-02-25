// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import javax.print.attribute.standard.RequestingUserName;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeHold extends SubsystemBase {
  
  private SparkMax algaeHoldMotor;
  private SparkMaxConfig algaeHoldConfig;
  private DigitalInput algaeHoldLimit;
  private boolean isAHoldException;

  public AlgaeHold() {
    algaeHoldMotor = new SparkMax(Constants.MotorControllers.ID_ALGAE_HOLD, MotorType.kBrushless);
    algaeHoldConfig = new SparkMaxConfig();
    
    algaeHoldConfig.inverted(false);
    algaeHoldConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    algaeHoldMotor.configure(algaeHoldConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);

    try {
      algaeHoldLimit = new DigitalInput(Constants.AlgaeHold.DIO_AH_LIMIT);
    } catch (Exception e)
    {
      isAHoldException = true;
      SmartDashboard.putBoolean("AlgaeHold Limit switch threw an exception", true);
    }
}

//METHODS Start Here
public boolean isAHoldLimit(){ //Leave normally open
  if (isAHoldException){
    return true; } 
  else{
    return !algaeHoldLimit.get(); //TODO: Change back for actual switch (temporarily inverted
  }
 }

 public boolean getAHoldLimit() {
    return !algaeHoldLimit.get(); //TODO: Change back for actual switch (temporarily inverted)
 }

  public void stopAlgaeHold(){
    algaeHoldMotor.set(0);
  }


  public void setAlgaeHoldSpeed(double speed1, double speed2)
  { // algaeHoldMotor.set(speed);
 
    if (isAHoldLimit()){
      algaeHoldMotor.set(speed2);
    } else {
      algaeHoldMotor.set(speed1);
    }
    }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("AlgaeHold limit:", isAHoldLimit());
  }
}