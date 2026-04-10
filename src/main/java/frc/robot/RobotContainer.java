// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.HoodSubsystem;
import frc.robot.subsystems.IndexSubsystem;

import java.util.List;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.TurretSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import edu.wpi.first.wpilibj.DriverStation;


public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final DriveSubsystem m_robotDrive = new DriveSubsystem();
  private final IntakeSubsystem m_intakeSubsystem = new IntakeSubsystem();
  private final IndexSubsystem m_indexSubsystem = new IndexSubsystem();
  private final TurretSubsystem m_turretSubsystem = new TurretSubsystem();
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(m_robotDrive);
  private final HoodSubsystem m_hoodSubsystem = new HoodSubsystem();
  private final LEDSubsystem m_ledSubsystem = new LEDSubsystem();


  private final SendableChooser<Command> autoChooser;

  XboxController m_driverController = new XboxController(OIConstants.kDriverControllerPort);

  public RobotContainer() {

    NamedCommands.registerCommand("extendIntake", 
        Commands.runOnce(() -> m_intakeSubsystem.extendIntake(), m_intakeSubsystem)
    );

    NamedCommands.registerCommand("retractIntake", 
        Commands.runOnce(() -> m_intakeSubsystem.retractIntake(), m_intakeSubsystem)
    );

    // 2. Build the auto chooser SECOND
    autoChooser = AutoBuilder.buildAutoChooser();

    // 3. Push it to the dashboard
    SmartDashboard.putData("Auto Choices", autoChooser);

    configureButtonBindings();

    m_robotDrive.setDefaultCommand(
        // The left stick controls translation of the robot.
        // Turning is controlled by the X axis of the right stick.
        new RunCommand(
            () -> m_robotDrive.drive(
                -MathUtil.applyDeadband(m_driverController.getLeftY(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getLeftX(), OIConstants.kDriveDeadband),
                -MathUtil.applyDeadband(m_driverController.getRightX(), OIConstants.kDriveDeadband),
                true),
            m_robotDrive));
}


// Define both target locations (You will need to measure/find the exact Red X/Y!)

// ==========================================
  // FIELD TARGETS (You must tune these X/Y values!)
  // ==========================================
  private final Pose2d BLUE_HUB_POSE = new Pose2d(4.611624, 4.0132, new Rotation2d());
  private final Pose2d RED_HUB_POSE  = new Pose2d(11.9126, 4.0132, new Rotation2d()); // Example coordinates

  // Blue Alliance Trenches
  private final Pose2d BLUE_TRENCH_1 = new Pose2d(5.0, 2.0, new Rotation2d());
  private final Pose2d BLUE_TRENCH_2 = new Pose2d(5.0, 6.0, new Rotation2d());

  // Red Alliance Trenches
  private final Pose2d RED_TRENCH_1 = new Pose2d(11.5, 2.0, new Rotation2d());
  private final Pose2d RED_TRENCH_2 = new Pose2d(11.5, 6.0, new Rotation2d());

  // Standard FRC Field Length (in meters). Verify this matches your game manual!
  private final double FIELD_LENGTH_METERS = 16.54;

/**
 * Checks the Driver Station and returns the correct hub pose.
 */
/**
   * Dynamically evaluates the best target based on Alliance Color and Robot Position.
   */
  public Pose2d getDynamicTargetPose() {
      var alliance = DriverStation.getAlliance();
      boolean isRed = alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;

      // 1. Get the robot's current position from the Drive Subsystem
      Pose2d currentPose = m_robotDrive.getPose();

      // 2. Calculate distance from our own alliance wall
      // Assuming standard WPILib origin where X=0 is Blue Wall, X=16.54 is Red Wall
      double distanceFromWall = isRed ? (FIELD_LENGTH_METERS - currentPose.getX()) : currentPose.getX();

      // 3. Make the decision
      if (distanceFromWall > 4.7) {
          // We are far away. Target the closer of our two trenches.
          Pose2d trench1 = isRed ? RED_TRENCH_1 : BLUE_TRENCH_1;
          Pose2d trench2 = isRed ? RED_TRENCH_2 : BLUE_TRENCH_2;

          // Calculate direct line-of-sight distance to both trenches
          double distToTrench1 = currentPose.getTranslation().getDistance(trench1.getTranslation());
          double distToTrench2 = currentPose.getTranslation().getDistance(trench2.getTranslation());

          // Return whichever trench is physically closer to the robot
          return distToTrench1 < distToTrench2 ? trench1 : trench2;
      } else {
          // We are close to the wall. Target our Hub.
          return isRed ? RED_HUB_POSE : BLUE_HUB_POSE;
      }
  }

  public double getDistanceToHub() {
    return m_robotDrive.getPose().getTranslation()
            .getDistance(getDynamicTargetPose().getTranslation());
}



  private void configureButtonBindings() {
    // new JoystickButton(m_driverController, XboxController.Button.kX.value)
    //     .whileTrue(new RunCommand(
    //         () -> m_robotDrive.setX(),
    //         m_robotDrive));

    new JoystickButton(m_driverController, XboxController.Button.kStart.value)
        .onTrue(new InstantCommand(
            () -> m_robotDrive.zeroHeading(),
            m_robotDrive));

    // ==========================================
    // If things go bad, uncomment this and comment the rest
    // ==========================================

// Button A: Run Intake and Index One Forward
// new JoystickButton(m_driverController, XboxController.Button.kA.value)
//     .whileTrue(new StartEndCommand(
//         () -> {
//             m_hoodSubsystem.raiseHood();
//         },  
//         () -> {
//             m_hoodSubsystem.lowerHood();
//         },
//         m_hoodSubsystem 
//     ));


    // ==========================================
    // BUTTON 'Y' - Auto-Aim, Shoot, & Dynamic Hood
    // ==========================================
    new JoystickButton(m_driverController, XboxController.Button.kY.value).whileTrue(
        Commands.parallel(
            // TASK 1: DRIVE AND SMART AIM
            m_robotDrive.driveAndAimCommand(
                () -> -m_driverController.getLeftY(), 
                () -> -m_driverController.getLeftX(), 
                this::getDynamicTargetPose 
            ),

            // TASK 2: CONTINUOUS DYNAMIC HOOD TRACKING (UPDATED)
            Commands.run(() -> {
                Pose2d currentTarget = getDynamicTargetPose();
                
                // Check if our current target is one of the Hubs
                if (currentTarget.equals(BLUE_HUB_POSE) || currentTarget.equals(RED_HUB_POSE)) {
                    // We are targeting the Hub! Calculate distance and use the table.
                    double distanceToTarget = m_robotDrive.getPose().getTranslation()
                                                .getDistance(currentTarget.getTranslation());
                    
                    double targetHoodPos = m_hoodSubsystem.calculateInterpolatedHoodPosition(distanceToTarget);
                    m_hoodSubsystem.setHoodPosition(targetHoodPos);
                } else {
                    // We are targeting a Trench! Lock the hood to 0.38
                    m_hoodSubsystem.setHoodPosition(0.38);
                }
            }, m_hoodSubsystem),

            // TASK 3: INTAKE & SHOOT SEQUENCE
            Commands.sequence(
                Commands.runOnce(() -> {
                    m_intakeSubsystem.extendIntake();
                    m_intakeSubsystem.runIntakeForward(0.8);
                    m_indexSubsystem.runIndexOneForward(0.8); 
                    m_turretSubsystem.shootRPM(6000);         
                }, m_intakeSubsystem, m_indexSubsystem, m_turretSubsystem),
                
                Commands.waitSeconds(1.0),
                
                Commands.run(() -> {
                    m_indexSubsystem.runIndexOneForward(0.3); 
                    m_indexSubsystem.runIndexTwoReverse(1.0); 
                }, m_indexSubsystem)
            )
            .finallyDo(() -> {
                m_intakeSubsystem.retractIntake();
                m_intakeSubsystem.runIntakeForward(0.0);
                m_turretSubsystem.stopShooter();
                m_indexSubsystem.runIndexTwoReverse(0.0);
                m_hoodSubsystem.lowerHood();
                
                Commands.waitSeconds(4.0)
                    .andThen(Commands.runOnce(() -> m_indexSubsystem.runIndexOneForward(0.0), m_indexSubsystem))
                    .schedule(); 
            })
        )
    );

    // ==========================================
    // BUTTON 'A' - Increase closest hood values (+0.03)
    // ==========================================
    new JoystickButton(m_driverController, XboxController.Button.kA.value).onTrue(
        Commands.runOnce(() -> {
            double currentDist = getDistanceToHub();
            m_hoodSubsystem.adjustClosestTableValues(currentDist, 0.01);
        })
    );

    // ==========================================
    // BUTTON 'B' - Decrease closest hood values (-0.03)
    // ==========================================
    new JoystickButton(m_driverController, XboxController.Button.kB.value).onTrue(
        Commands.runOnce(() -> {
            double currentDist = getDistanceToHub();
            m_hoodSubsystem.adjustClosestTableValues(currentDist, -0.01);
        })
    );


// new JoystickButton(m_driverController, XboxController.Button.kB.value).whileTrue(m_robotDrive.driveToPoseCommand(new Pose2d(1.0, 1.0, Rotation2d.fromDegrees(0))));
    
// ==========================================
    // BUTTON 'X' - Auto-Aim, Shoot, & Smart Hood (NO INTAKE)
    // ==========================================
    new JoystickButton(m_driverController, XboxController.Button.kX.value).whileTrue(
        Commands.parallel(
            // TASK 1: DRIVE AND SMART AIM
            m_robotDrive.driveAndAimCommand(
                () -> -m_driverController.getLeftY(), 
                () -> -m_driverController.getLeftX(), 
                this::getDynamicTargetPose 
            ),

            // TASK 2: CONTINUOUS DYNAMIC HOOD TRACKING
            Commands.run(() -> {
                Pose2d currentTarget = getDynamicTargetPose();
                
                // Check if our current target is one of the Hubs
                if (currentTarget.equals(BLUE_HUB_POSE) || currentTarget.equals(RED_HUB_POSE)) {
                    // We are targeting the Hub! Calculate distance and use the table.
                    double distanceToTarget = m_robotDrive.getPose().getTranslation()
                                                .getDistance(currentTarget.getTranslation());
                    
                    double targetHoodPos = m_hoodSubsystem.calculateInterpolatedHoodPosition(distanceToTarget);
                    m_hoodSubsystem.setHoodPosition(targetHoodPos);
                } else {
                    // We are targeting a Trench! Lock the hood to 0.38
                    m_hoodSubsystem.setHoodPosition(0.38);
                }
            }, m_hoodSubsystem),

            // TASK 3: SHOOT SEQUENCE ONLY (No Intake)
            Commands.sequence(
                Commands.runOnce(() -> {
                    m_indexSubsystem.runIndexOneForward(0.8); // Shift note up
                    m_turretSubsystem.shootRPM(6000);         // Spool up
                }, m_indexSubsystem, m_turretSubsystem),      // <-- Removed m_intakeSubsystem requirement!
                
                Commands.waitSeconds(1.0),
                
                Commands.run(() -> {
                    m_indexSubsystem.runIndexOneForward(0.3); 
                    m_indexSubsystem.runIndexTwoReverse(1.0); 
                }, m_indexSubsystem)
            )
            .finallyDo(() -> {
                // <-- Removed intake retract commands from here
                m_turretSubsystem.stopShooter();
                m_indexSubsystem.runIndexTwoReverse(0.0);
                m_hoodSubsystem.lowerHood();
                
                Commands.waitSeconds(4.0)
                    .andThen(Commands.runOnce(() -> m_indexSubsystem.runIndexOneForward(0.0), m_indexSubsystem))
                    .schedule(); 
            })
        )
    );

    // ==========================================
    // BUTTON 'RIGHT BUMPER' - Shoot
    // ==========================================

    new JoystickButton(m_driverController, XboxController.Button.kRightBumper.value).whileTrue(
    // 1. Start the shooter immediately
    Commands.runOnce(() -> {
        m_turretSubsystem.shootRPM(6000);
    }, m_turretSubsystem)     // <-- Added m_hoodSubsystem as a requirement)
        
        // 2. Wait exactly 1.0 seconds
        .andThen(Commands.waitSeconds(1.0))
        
        // 3. Turn on the indexer to feed the note
        .andThen(Commands.run(() -> {
            m_indexSubsystem.runIndexOneForward(0.3);
            m_indexSubsystem.runIndexTwoReverse(1);
        }, m_indexSubsystem))
        
        // 4. When the button is released (or interrupted), stop EVERYTHING
        .finallyDo(() -> {
            m_indexSubsystem.runIndexOneForward(0);
            m_indexSubsystem.runIndexTwoReverse(0);
            m_turretSubsystem.stopShooter();
        })
);

    // ==========================================
    // BUTTON 'LEFT BUMPER' - Intake
    // ==========================================

// When Left Bumper is HELD: Extend the intake AND spin the rollers.
// When RELEASED: Retract the intake AND stop the rollers.
// When Left Bumper is HELD: Extend, spin intake, and start indexer.
// When RELEASED: Retract, stop intake, and schedule the indexer to stop 4s later.
new JoystickButton(m_driverController, XboxController.Button.kLeftBumper.value).whileTrue(
    new StartEndCommand(
        () -> {
            // --- RUNS ONCE ON PRESS ---
            m_intakeSubsystem.extendIntake();
            m_intakeSubsystem.runIntakeForward(0.8); 
            m_indexSubsystem.runIndexOneForward(0.8); // Start indexer immediately
        },  
        () -> {
            // --- RUNS ONCE ON RELEASE ---
            m_intakeSubsystem.retractIntake();
            m_intakeSubsystem.runIntakeForward(0.0); 

            // Fire-and-forget command: Wait 4 seconds, then stop the indexer
            new WaitCommand(4.0)
                .andThen(() -> m_indexSubsystem.runIndexOneForward(0.0), m_indexSubsystem)
                .schedule(); // <--- This tells WPILib to run this sequence in the background!
        },
        m_intakeSubsystem, m_indexSubsystem // Require BOTH subsystems
    )
);
    
}

public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }

}