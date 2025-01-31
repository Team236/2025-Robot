// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.swerve.SwerveModuleConstants.ClosedLoopOutputType;
import com.revrobotics.RelativeEncoder;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.config.SparkBaseConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

//TODO change to using a brushed motor, with SparkMax controller 
//TODO also change to external encoder (see 2024 front drive motors - 2 DIO channels A/B needed)

public class CoralPivot extends SubsystemBase {
  
  private SparkMax coralPivotMotor;
  private SparkBaseConfig coralPivotConfig;
  //private RelativeEncoder coralPivotEncoder;
  private Encoder coralPivotEncoder;
  private boolean isCoralPivotExtException, isCoralPivotRetException;
  private DigitalInput CoralExtLimit, CoralRetLimit;
 

    /** Creates a new CoralPivot. */
    public CoralPivot() {
    coralPivotMotor = new SparkMax(Constants.MotorControllers.ID_CORAL_PIVOT, MotorType.kBrushless);
    //coralPivotEncoder = coralPivotMotor.getEncoder(); 

    coralPivotEncoder = new Encoder(Constants.CoralPivot.DIO_ENC_A, Constants.CoralPivot.DIO_ENC_B);

    coralPivotConfig = new SparkMaxConfig();
    coralPivotConfig.inverted(false); 
    coralPivotConfig.smartCurrentLimit(Constants.MotorControllers.SMART_CURRENT_LIMIT);

    coralPivotMotor.configure(coralPivotConfig,SparkBase.ResetMode.kResetSafeParameters ,SparkBase.PersistMode.kPersistParameters);
        
    try {
      //  This tries to make a new digital input, and if it fails, throws an error 
      CoralExtLimit = new DigitalInput(Constants.CoralPivot.DIO_EXT_LIMIT);
    } catch (Exception e) {
       isCoralPivotExtException = true;
      SmartDashboard.putBoolean("exception thrown for Coral Extend limit: ", isCoralPivotExtException);
    }
    try {
      //  This sets a bottom limit for the coral, and if it fails, throws an error
      CoralRetLimit = new DigitalInput(Constants.CoralPivot.DIO_RET_LIMIT);
    } catch (Exception e) {
      isCoralPivotRetException = true;
      SmartDashboard.putBoolean("exception thrown for Coral Retract limit: ", isCoralPivotRetException);
    }
}

// methods start here
public double getCoralEncoder() {  //gives encoder reading in Revs
  //return coralPivotEncoder.getPosition();
  return coralPivotEncoder.getRaw() / 256;
}

public void resetCoralEncoder() {
  coralPivotEncoder.reset();
}

public void stopCoralPivot() {
  coralPivotMotor.set(0);
}

public double getCoralPivotSpeed() {
  return coralPivotMotor.get();
}
public boolean isCoralExtLimit() {
if (isCoralPivotExtException) {
  return true;
} else {
  return CoralExtLimit.get();
}
}

public boolean isCoralRetLimit() {
if (isCoralPivotRetException) {
  return true;
} else {
  return CoralRetLimit.get();
}
}

public boolean isFullyExtended() {
  return (getCoralEncoder() <= Constants.CoralPivot.ENC_REVS_MAX);
}

public void setCoralPivotSpeed(double speed) {
  if (speed <= 0) {  
    if (isCoralExtLimit() || isFullyExtended()) {
        // if fully extended limit is tripped or at the maximum desired extension and going out, stop 
        stopCoralPivot();
     }  else {
        // extending out but fully extended limit is not tripped, go at commanded speed
       coralPivotMotor.set(speed);
      }
 } 
 else {
      if (isCoralRetLimit()) {
        // retracting and retract limit is tripped, stop and zero encoder
        stopCoralPivot();
        resetCoralEncoder();
      } else {
        // retracting but fully retracted limit is not tripped, go at commanded speed
        coralPivotMotor.set(speed);
      }
     }
}



//Begin things that may not be relevant
//these are things that might be useful in the future if we use CANSparkMax PID
//we are not currently using it

//!!!! SPARKMAX PID STUFF - USE SPARKMAX PID, NOT WPILib PID 
//**** CHANGED BACK TO USING WPILib PID ****
//**** due to spurious encoder polarity changes when run multiple autos in a row ****
/*
public void setSetpoint(double encoderRevs) {
  tiltPIDController.setReference(encoderRevs, ControlType.kPosition);
}

public void setP(double kP) {
  tiltPIDController.setP(kP);
}

public void setI(double kI) {
  tiltPIDController.setI(kI);
}

public void setD(double kD) {
  tiltPIDController.setD(kD);
}

public void setFF(double kFF) {
  tiltPIDController.setFF(kFF);
}
*/


@Override
  public void periodic() {
    SmartDashboard.putBoolean("Coral Pivot Extend Limit: ", isCoralExtLimit());
    SmartDashboard.putBoolean("Coral Pivot Retract Limit", isCoralRetLimit());
    SmartDashboard.putNumber("Coral Pivot Encoder Revolutions ", getCoralEncoder());
    SmartDashboard.putBoolean("Coral Pivot is fully extended: ", isFullyExtended());
  }

}