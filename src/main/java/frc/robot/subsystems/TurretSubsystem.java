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

// ======== NEW IMPORTS FOR VELOCITY CONTROL ========
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.config.ClosedLoopConfig;
// ADD THIS LINE
import com.revrobotics.spark.SparkBase.ControlType;
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

  // ======== NEW: CLOSED LOOP CONTROLLERS ========
  private SparkClosedLoopController shooterControllerOne;
  private SparkClosedLoopController shooterControllerTwo;

  

  public TurretSubsystem() {

    // ==========================================
    // 1. SHOOTER FLYWHEEL INITIALIZATION
    // ==========================================
    turretMotorOne = new SparkFlex(Constants.TurretConstants.turretMotorOneID, MotorType.kBrushless);
    turretMotorTwo = new SparkFlex(Constants.TurretConstants.turretMotorTwoID, MotorType.kBrushless);

    // Fetch the closed loop controllers directly from the SparkFlex objects
    shooterControllerOne = turretMotorOne.getClosedLoopController();
    shooterControllerTwo = turretMotorTwo.getClosedLoopController();

    SparkFlexConfig turretConfig = new SparkFlexConfig();
    turretConfig.idleMode(IdleMode.kCoast);
    // Bumped current limit slightly to handle the aggressive spin-up of velocity control
    turretConfig.smartCurrentLimit(40); 
    turretConfig.secondaryCurrentLimit(45);
    turretConfig.voltageCompensation(12.0);

    // --- Configure the Velocity PID & Feedforward ---
    ClosedLoopConfig closedLoopConfig = new ClosedLoopConfig();
    closedLoopConfig.p(0.01); // Start very small for flywheels
    closedLoopConfig.i(0.0);
    closedLoopConfig.d(0.0);
    // Feedforward (FF) does the heavy lifting. 
    // Roughly 1 / Max_RPM. (e.g., if Vortex max RPM is 6700, 1/6700 = ~0.00015)
    closedLoopConfig.velocityFF(0.00015); 

    // Attach the closed loop settings to our main config
    turretConfig.apply(closedLoopConfig);

    // Apply to Motor One
    turretMotorOne.configure(turretConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // For Motor Two, we copy the exact same config, but invert it so it spins the opposite way!
    SparkFlexConfig turretTwoConfig = new SparkFlexConfig();
    turretTwoConfig.apply(turretConfig);
    turretTwoConfig.inverted(true);
    turretMotorTwo.configure(turretTwoConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


  }

  // ==========================================
  // SHOOTER COMMANDS
  // ==========================================
  
  /**
   * Tell the flywheels to reach and maintain a specific RPM
   */
  public void shootRPM(double targetRPM) {
    // Note: We use .setSetpoint() for the 2025 REVLib API!
    shooterControllerOne.setSetpoint(targetRPM, ControlType.kVelocity);
    shooterControllerTwo.setSetpoint(targetRPM, ControlType.kVelocity);
  }

  /**
   * Stop the shooter motors safely
   */
  public void stopShooter() {
    turretMotorOne.set(0);
    turretMotorTwo.set(0);
  }

}