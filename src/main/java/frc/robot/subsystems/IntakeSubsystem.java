// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.security.cert.X509CRLSelector;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
  /** Creates a new IntakeSubsystem. */

  //Motor - spin or not or reverse
  //NeoV2

  //The plan

  /*Configure our motor:
    By default, it should brake
      On button press, it should spin
      On button press, it should spin the other way
  */
  

  private SparkMax intakeMotor;

  public IntakeSubsystem() {

    //Configure the intake
    intakeMotor = new SparkMax(Constants.IntakeConstants.intakeMotorID,MotorType.kBrushless);

    SparkMaxConfig intakeConfig = new SparkMaxConfig();

    //  Find the needed parameters
    // I added the most common ones that all FRC teams use, but the values may need to change

    //Set the brake for when not receiving a command
    intakeConfig.idleMode(IdleMode.kBrake);

    //Smart cutoff, the motor will try to keep this amount of Amperes
    intakeConfig.smartCurrentLimit(35);
    //The main cutoff - it will never go above it
    intakeConfig.secondaryCurrentLimit(50);
    //Stabilizes the torgue so that when the battery is fresh it works the same way as if it's not
    intakeConfig.voltageCompensation(12.0);


    //This line makes sure the settings are persistant and that we reset the controller to safe defaults
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  //Most basic

  // public Command runIntake(){
  //   return run(() -> intakeMotor.set(1));
  // }

  // public Command runIntakeReverse(){
  //   return run(() -> intakeMotor.set(-1));
  // }


  //With voltage

  // public Command runIntake(){
  //   return run(() -> intakeMotor.setVoltage(12));
  // }

  // public Command runIntakeReverse(){
  //   return run(() -> intakeMotor.setVoltage(-12));
  // }


  //With toggles

  public Command runIntake(){
    return runEnd(
        () -> intakeMotor.setVoltage(12),
        () -> intakeMotor.setVoltage(0));
  }

  public Command runIntakeReverse(){
    return runEnd(
        () -> intakeMotor.setVoltage(-12),
        () -> intakeMotor.setVoltage(0)); //technically we don't need this
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
