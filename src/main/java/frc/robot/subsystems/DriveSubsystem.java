// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj.DriverStation;

// --- PATHPLANNER IMPORTS ---
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;

public class DriveSubsystem extends SubsystemBase {
  // Create MAXSwerveModules
  private final MAXSwerveModule m_frontLeft = new MAXSwerveModule(
      DriveConstants.kFrontLeftDrivingCanId,
      DriveConstants.kFrontLeftTurningCanId,
      DriveConstants.kFrontLeftChassisAngularOffset);

  private final MAXSwerveModule m_frontRight = new MAXSwerveModule(
      DriveConstants.kFrontRightDrivingCanId,
      DriveConstants.kFrontRightTurningCanId,
      DriveConstants.kFrontRightChassisAngularOffset);

  private final MAXSwerveModule m_rearLeft = new MAXSwerveModule(
      DriveConstants.kRearLeftDrivingCanId,
      DriveConstants.kRearLeftTurningCanId,
      DriveConstants.kBackLeftChassisAngularOffset);

  private final MAXSwerveModule m_rearRight = new MAXSwerveModule(
      DriveConstants.kRearRightDrivingCanId,
      DriveConstants.kRearRightTurningCanId,
      DriveConstants.kBackRightChassisAngularOffset);

  private final AHRS m_gyro = new AHRS(NavXComType.kMXP_SPI);

  // Pose Estimator class for tracking robot pose (replaces Odometry)
  SwerveDrivePoseEstimator m_poseEstimator = new SwerveDrivePoseEstimator(
      DriveConstants.kDriveKinematics,
      Rotation2d.fromDegrees(-m_gyro.getAngle()),
      new SwerveModulePosition[] {
          m_frontLeft.getPosition(),
          m_frontRight.getPosition(),
          m_rearLeft.getPosition(),
          m_rearRight.getPosition()
      },
      new Pose2d() // Initial starting pose
  );

  private final Field2d m_field = new Field2d();

  /** Creates a new DriveSubsystem. */
  public DriveSubsystem() {
    // Usage reporting for MAXSwerve template
    HAL.report(tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_MaxSwerve);

    // --- PATHPLANNER CONFIGURATION ---
    try {
      // Load robot physical properties from the PathPlanner GUI settings
      RobotConfig config = RobotConfig.fromGUISettings();

      AutoBuilder.configure(
          this::getPose, 
          this::resetOdometry, 
          this::getChassisSpeeds, 
          this::driveRobotRelative, 
          new PPHolonomicDriveController(
              new PIDConstants(5.0, 0.0, 0.0), // Translation PID (Tune these!)
              new PIDConstants(5.0, 0.0, 0.0)  // Rotation PID (Tune these!)
          ),
          config,
          () -> {
              // Automatically flip paths if we are on the Red Alliance
              var alliance = DriverStation.getAlliance();
              if (alliance.isPresent()) {
                  return alliance.get() == DriverStation.Alliance.Red;
              }
              return false;
          },
          this // Requires this subsystem
      );
    } catch (Exception e) {
      DriverStation.reportError("Failed to load PathPlanner config! Ensure your GUI settings are saved.", e.getStackTrace());
    }
  }

  @Override
  public void periodic() {
    // Update the pose estimator with encoder and gyro data
    m_poseEstimator.update(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        });
    SmartDashboard.putNumber("Gyro Angle", -m_gyro.getAngle());
    
    SmartDashboard.putData("Field", m_field);
    m_field.setRobotPose(m_poseEstimator.getEstimatedPosition());

    // Optional but highly recommended: send your exact position to the dashboard!
    Pose2d currentPose = getPose();
    SmartDashboard.putNumber("Robot X", currentPose.getX());
    SmartDashboard.putNumber("Robot Y", currentPose.getY());
  
    Pose2d towerPose = new Pose2d(8.0, 4.0, new Rotation2d());

    // Update the dashboard with targeting data every 20ms
    outputTargetingData(towerPose);
  }

  public Pose2d getPose() {
    return m_poseEstimator.getEstimatedPosition();
  }

  public void resetOdometry(Pose2d pose) {
    m_poseEstimator.resetPosition(
        Rotation2d.fromDegrees(-m_gyro.getAngle()),
        new SwerveModulePosition[] {
            m_frontLeft.getPosition(),
            m_frontRight.getPosition(),
            m_rearLeft.getPosition(),
            m_rearRight.getPosition()
        },
        pose);
  }
  
  public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
      m_poseEstimator.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
  }

  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    double xSpeedDelivered = xSpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double ySpeedDelivered = ySpeed * DriveConstants.kMaxSpeedMetersPerSecond;
    double rotDelivered = rot * DriveConstants.kMaxAngularSpeed;

    var swerveModuleStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(
        fieldRelative
            ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered, Rotation2d.fromDegrees(-m_gyro.getAngle()))
            : new ChassisSpeeds(xSpeedDelivered, ySpeedDelivered, rotDelivered));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_rearLeft.setDesiredState(swerveModuleStates[2]);
    m_rearRight.setDesiredState(swerveModuleStates[3]);
  }

  public void setX() {
    m_frontLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
    m_frontRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearLeft.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(-45)));
    m_rearRight.setDesiredState(new SwerveModuleState(0, Rotation2d.fromDegrees(45)));
  }

  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
    m_frontLeft.setDesiredState(desiredStates[0]);
    m_frontRight.setDesiredState(desiredStates[1]);
    m_rearLeft.setDesiredState(desiredStates[2]);
    m_rearRight.setDesiredState(desiredStates[3]);
  }

  public void resetEncoders() {
    m_frontLeft.resetEncoders();
    m_rearLeft.resetEncoders();
    m_frontRight.resetEncoders();
    m_rearRight.resetEncoders();
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public double getHeading() {
    return Rotation2d.fromDegrees(-m_gyro.getAngle()).getDegrees();
  }

  public double getTurnRate() {
    return m_gyro.getRate() * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
  }

  public void outputTargetingData(Pose2d targetPose) {
    Pose2d currentPose = getPose();
    double distanceToTower = currentPose.getTranslation().getDistance(targetPose.getTranslation());
    Translation2d difference = targetPose.getTranslation().minus(currentPose.getTranslation());
    Rotation2d targetAngle = difference.getAngle();
    Rotation2d rotationError = targetAngle.minus(currentPose.getRotation());
    double turnAmountDegrees = rotationError.getDegrees();
    boolean turnLeft = turnAmountDegrees > 0;
    double absoluteTurnAmount = Math.abs(turnAmountDegrees);

    SmartDashboard.putBoolean("Targeting/Turn Left", turnLeft);
    SmartDashboard.putNumber("Targeting/Turn Amount (Deg)", absoluteTurnAmount);
    SmartDashboard.putNumber("Targeting/Distance to Tower (Meters)", distanceToTower);
  }

  public Command driveToPoseCommand(Pose2d targetPose) {
    PIDController xController = new PIDController(1.0, 0, 0);
    PIDController yController = new PIDController(1.0, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
        1.0, 0, 0, new TrapezoidProfile.Constraints(Math.PI, Math.PI));

    thetaController.enableContinuousInput(-Math.PI, Math.PI);
    xController.setTolerance(0.05);
    yController.setTolerance(0.05);
    thetaController.setTolerance(Math.toRadians(2));

    return this.run(() -> {
      Pose2d currentPose = getPose();
      double xSpeed = MathUtil.clamp(xController.calculate(currentPose.getX(), targetPose.getX()), -1.0, 1.0);
      double ySpeed = MathUtil.clamp(yController.calculate(currentPose.getY(), targetPose.getY()), -1.0, 1.0);
      double rotSpeed = MathUtil.clamp(
          thetaController.calculate(currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()), 
          -1.0, 1.0);
      drive(xSpeed, ySpeed, rotSpeed, true);
    })
    .until(() -> xController.atSetpoint() && yController.atSetpoint() && thetaController.atGoal())
    .finallyDo(() -> drive(0, 0, 0, true));
  }
// 1. CHANGE THIS LINE: Accept a Supplier<Pose2d> instead of a static Pose2d
  public Command driveAndAimCommand(DoubleSupplier xSupplier, DoubleSupplier ySupplier, Supplier<Pose2d> targetPoseSupplier) {
    
    // Create a rotation PID controller to snap to the target angle
    ProfiledPIDController thetaController = new ProfiledPIDController(
        1.5, 0, 0, 
        new TrapezoidProfile.Constraints(Math.PI * 2, Math.PI * 2) 
    );
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    return this.run(() -> {
      double xSpeed = xSupplier.getAsDouble();
      double ySpeed = ySupplier.getAsDouble();
      Pose2d currentPose = getPose();

      // ========================================================
      // 2. ADD THIS LINE: Fetch the correct pose RIGHT NOW.
      // This forces the code to check your alliance color 
      // every single loop while you hold the button!
      // ========================================================
      Pose2d activeTargetPose = targetPoseSupplier.get();

      // Calculate the angle required to point AT the target using the activeTargetPose
      Translation2d difference = activeTargetPose.getTranslation().minus(currentPose.getTranslation());
      Rotation2d targetAngle = difference.getAngle();

      // Calculate the rotation speed using PID to close the gap
      double rotSpeed = thetaController.calculate(
          currentPose.getRotation().getRadians(), 
          targetAngle.getRadians()
      );
      rotSpeed = MathUtil.clamp(rotSpeed, -1.0, 1.0);

      // Drive! (Feed joystick translation + automatic rotation)
      drive(xSpeed, ySpeed, rotSpeed, true);
    });
  }

  // --- NEW PATHPLANNER HELPER METHODS ---

  /**
   * Returns an array of the current state of each swerve module.
   */
  public SwerveModuleState[] getModuleStates() {
      return new SwerveModuleState[] {
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_rearLeft.getState(),
          m_rearRight.getState()
      };
  }

  /**
   * Returns the current overall chassis speeds of the robot.
   */
  public ChassisSpeeds getChassisSpeeds() {
      return DriveConstants.kDriveKinematics.toChassisSpeeds(getModuleStates());
  }

  /**
   * Drives the robot directly using raw ChassisSpeeds without joystick scaling.
   * PathPlanner uses this to send direct robot-relative velocity vectors.
   */
  public void driveRobotRelative(ChassisSpeeds speeds) {
      SwerveModuleState[] desiredStates = DriveConstants.kDriveKinematics.toSwerveModuleStates(speeds);
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, DriveConstants.kMaxSpeedMetersPerSecond);
      m_frontLeft.setDesiredState(desiredStates[0]);
      m_frontRight.setDesiredState(desiredStates[1]);
      m_rearLeft.setDesiredState(desiredStates[2]);
      m_rearRight.setDesiredState(desiredStates[3]);
  }
}