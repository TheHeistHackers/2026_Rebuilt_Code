package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.PersistMode;
import com.revrobotics.ResetMode;


import frc.robot.Constants;


public class HoodSubsystem extends SubsystemBase{
    // Hood Motor & Sensors



  // Flag for periodic loop
  private boolean isMovingHood = false;
  private SparkMax hoodMotor; 
  private final DutyCycleEncoder hoodEncoder;
  private final PIDController hoodController;

  
  public HoodSubsystem(){

  
    // ==========================================
    // 2. HOOD ENCODER & PID INITIALIZATION
    // ==========================================
    hoodMotor = new SparkMax(Constants.TurretConstants.hoodMotorID, MotorType.kBrushless);

    hoodEncoder = new DutyCycleEncoder(
      Constants.TurretConstants.hoodEncoderDIOPort, 
      1.0, 
      Constants.TurretConstants.hoodEncoderOffset
    );
    
    hoodController = new PIDController(1.5, 0.0, 0.0);
    hoodController.setTolerance(0.01); 

    // ==========================================
    // 3. HOOD MOTOR CONFIGURATION
    // ==========================================
    SparkMaxConfig hoodConfig = new SparkMaxConfig();
    
    hoodConfig.idleMode(IdleMode.kBrake); 
    hoodConfig.smartCurrentLimit(20);     
    hoodConfig.inverted(false); 

    hoodMotor.configure(hoodConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }
  // ==========================================
  // HOOD COMMANDS
  // ==========================================
  public void raiseHood() {
    System.out.println("Raising Hood - max");
    hoodController.setSetpoint(0.38);
    isMovingHood = true;
  }

  public void lowerHood() {
    System.out.println("Lowering Hood");
    hoodController.setSetpoint(0.51);
    isMovingHood = true;
  }

  public void stopHood() {
    isMovingHood = false;
    hoodMotor.set(0);
  }

  @Override
  public void periodic() {
    // --- HOOD PID LOGIC ---
    double currentHoodPos = hoodEncoder.get(); 

    if (isMovingHood) {
      double motorPower = hoodController.calculate(currentHoodPos);

      if (hoodController.atSetpoint()) {
          stopHood();
      } else {
          hoodMotor.set(MathUtil.clamp(motorPower, -0.4, 0.4));
      }
    }

    // =========================================================
    // SMARTDASHBOARD DEBUGGING
    // =========================================================
    
    // Hood Data
    SmartDashboard.putNumber("Hood/Current_Position", currentHoodPos);
    SmartDashboard.putBoolean("Hood/Is_Moving", isMovingHood);
    SmartDashboard.putNumber("Hood/Target_Setpoint", hoodController.getSetpoint());
    SmartDashboard.putNumber("Hood/Position_Error", hoodController.getPositionError());
    SmartDashboard.putNumber("Hood/Applied_Motor_Power", hoodMotor.get());
  }
}
