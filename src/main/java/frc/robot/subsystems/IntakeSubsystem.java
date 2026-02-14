// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

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
    //Set the brake for when not receiving a command
    intakeConfig.idleMode(IdleMode.kBrake);

    intakeConfig.smartCurrentLimit(35);
    intakeConfig.secondaryCurrentLimit(50);
    intakeConfig.voltageCompensation(12.0);


    //This line makes sure the settings are persistant and that we reset the controller to safe defaults
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
