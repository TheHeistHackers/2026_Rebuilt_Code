// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax; // Added for the hood motor
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig; // Added for the hood config
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class TurretSubsystem extends SubsystemBase {

  // Shooter Motors (Flywheels)
  private SparkFlex turretMotorOne;
  private SparkFlex turretMotorTwo;

  // Hood Motor & Sensors
  // Assuming a SparkMax for the hood. Change to SparkFlex if using a Vortex!
  private SparkMax hoodMotor; 
  private final DutyCycleEncoder hoodEncoder;
  private final PIDController hoodController;

  // Flag for periodic loop
  private boolean isMovingHood = false;

  public TurretSubsystem() {

    // ==========================================
    // 1. SHOOTER FLYWHEEL INITIALIZATION
    // ==========================================
    turretMotorOne = new SparkFlex(Constants.TurretConstants.turretMotorOneID, MotorType.kBrushless);
    turretMotorTwo = new SparkFlex(Constants.TurretConstants.turretMotorTwoID, MotorType.kBrushless);

    SparkFlexConfig turretConfig = new SparkFlexConfig();
    turretConfig.idleMode(IdleMode.kCoast);
    turretConfig.smartCurrentLimit(35);
    turretConfig.secondaryCurrentLimit(39);
    turretConfig.voltageCompensation(12.0);

    turretMotorOne.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    turretMotorTwo.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // ==========================================
    // 2. HOOD ENCODER & PID INITIALIZATION
    // ==========================================
    // Initialize the hood motor
    hoodMotor = new SparkMax(Constants.TurretConstants.hoodMotorID, MotorType.kBrushless);

    // Initialize the absolute encoder
    hoodEncoder = new DutyCycleEncoder(
      Constants.TurretConstants.hoodEncoderDIOPort, 
      1.0, 
      Constants.TurretConstants.hoodEncoderOffset
    );
    
    // Initialize the PID Controller (Start conservative with P=1.0)
    hoodController = new PIDController(1.0, 0.0, 0.0);
    hoodController.setTolerance(0.02); // Tighter tolerance since hoods usually require precision

    // ==========================================
    // 3. HOOD MOTOR CONFIGURATION
    // ==========================================
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    
    hoodConfig.idleMode(IdleMode.kBrake); // Hold position firmly against gravity/vibrations
    hoodConfig.smartCurrentLimit(20);     // Hoods usually need less current, adjust as necessary
    
    // You may need to invert this depending on your physical gearing!
    hoodConfig.inverted(true); 

    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  // ==========================================
  // SHOOTER COMMANDS
  // ==========================================
  public void shoot(double speed){
    turretMotorOne.set(speed);
    turretMotorTwo.set(-speed);
  }
  
  public void shootReverse(double speed){
    turretMotorOne.set(-speed);
    turretMotorTwo.set(speed);
  }

  // ==========================================
  // HOOD COMMANDS
  // ==========================================
  public void raiseHood() {
    System.out.println("Raising Hood - max");
    hoodController.setSetpoint(0.95);
    isMovingHood = true;
  }

  public void lowerHood() {
    System.out.println("Lowering Hood");
    hoodController.setSetpoint(-0.05);
    isMovingHood = true;
  }

  public void stopHood() {
    isMovingHood = false;
    hoodMotor.set(0);
  }

  @Override
  public void periodic() {
    // 1. Always read the sensor
    double currentHoodPos = hoodEncoder.get(); 

    // 2. Handle PID Movement
    if (isMovingHood) {
      double motorPower = hoodController.calculate(currentHoodPos);

      if (hoodController.atSetpoint()) {
          stopHood();
      } else {
          // Clamp power for safety while tuning
          hoodMotor.set(MathUtil.clamp(motorPower, -0.4, 0.4));
      }
    }

    // =========================================================
    // SMARTDASHBOARD DEBUGGING (Organized into a "Hood" folder)
    // =========================================================
    SmartDashboard.putNumber("Hood/Current_Position", currentHoodPos);
    SmartDashboard.putBoolean("Hood/Is_Moving", isMovingHood);
    SmartDashboard.putNumber("Hood/Target_Setpoint", hoodController.getSetpoint());
    SmartDashboard.putNumber("Hood/Position_Error", hoodController.getPositionError());
    SmartDashboard.putBoolean("Hood/At_Setpoint", hoodController.atSetpoint());
    SmartDashboard.putNumber("Hood/Applied_Motor_Power", hoodMotor.get());
  }
}