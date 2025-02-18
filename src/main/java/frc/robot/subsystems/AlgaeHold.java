// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class AlgaeHold extends SubsystemBase {
  
  private SparkMax algaeHoldMotor;
  private SparkMaxConfig algaeHoldConfig;


  public AlgaeHold() {
    algaeHoldMotor = new SparkMax(Constants.MotorControllers.ID_ALGAE_HOLD, MotorType.kBrushless);

    algaeHoldConfig = new SparkMaxConfig();
    
    algaeHoldConfig.inverted(false);
    algaeHoldConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    algaeHoldMotor.configure(algaeHoldConfig, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
  }

  public void stopAlgaeHold()
  {
    algaeHoldMotor.set(0);
  }

  public void setAlgaeHoldSpeed(double speed)
  {
    algaeHoldMotor.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}