// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  private SparkFlex turretMotorOne;
  private SparkFlex turretMotorTwo;


  public TurretSubsystem() {

    //Configure the intake
    turretMotorOne = new SparkFlex(Constants.TurretConstants.turretMotorOneID,MotorType.kBrushless);
    turretMotorTwo = new SparkFlex(Constants.TurretConstants.turretMotorTwoID,MotorType.kBrushless);


    SparkFlexConfig turretConfig = new SparkFlexConfig();

    //  Find the needed parameters
    // I added the most common ones that all FRC teams use, but the values may need to change

    //Set the brake for when not receiving a command
    turretConfig.idleMode(IdleMode.kCoast);

    //Smart cutoff, the motor will try to keep this amount of Amperes
    turretConfig.smartCurrentLimit(35);
    //The main cutoff - it will never go above it. Hardware breaker is 40A, so set software breaker just below
    turretConfig.secondaryCurrentLimit(39);
    //Stabilizes the torgue so that when the battery is fresh it works the same way as if it's not
    turretConfig.voltageCompensation(12.0);


    //This line makes sure the settings are persistant and that we reset the controller to safe defaults
    turretMotorOne.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretMotorTwo.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void shoot(double speed){
    turretMotorOne.set(speed);
    turretMotorTwo.set(-speed);
  }

  
  public void shootReverse(double speed){
    turretMotorOne.set(-speed);
    turretMotorTwo.set(speed);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}