package frc.robot.subsystems;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import java.util.Optional;

public class VisionSubsystem extends SubsystemBase {
    
    private final PhotonCamera camera = new PhotonCamera("RoundedCamera"); 
    private final PhotonPoseEstimator poseEstimator;

    private final DriveSubsystem driveSubsystem;

    public VisionSubsystem(DriveSubsystem driveSubsystem) {
        this.driveSubsystem = driveSubsystem;
        Transform3d robotToCam = new Transform3d(
                new Translation3d(0.2, 0.0, 0.5), 
                new Rotation3d(0, 0, 0)
        );

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField);

        // The updated 2-argument constructor!
        poseEstimator = new PhotonPoseEstimator(fieldLayout, robotToCam);
    }

    @Override
    public void periodic() {
        // 1. Get the latest pipeline result from the camera
        var result = camera.getLatestResult();


        // 2. Feed that result directly into the multi-tag pose strategy
        Optional<EstimatedRobotPose> estimatedPose = poseEstimator.estimateCoprocMultiTagPose(result);

        if (estimatedPose.isEmpty()){
            estimatedPose = poseEstimator.estimateLowestAmbiguityPose(result);
        }

        // 3. If it successfully calculated a position
        if (estimatedPose.isPresent()) {
            Pose2d robotFieldPosition = estimatedPose.get().estimatedPose.toPose2d();

            // Extract the exact timestamp the picture was taken
            double imageCaptureTime = estimatedPose.get().timestampSeconds;

            // SEND THE DATA TO THE DRIVETRAIN!
            driveSubsystem.addVisionMeasurement(robotFieldPosition, imageCaptureTime);

            // Print it to SmartDashboard or Glass
            SmartDashboard.putNumber("Robot X (Meters)", robotFieldPosition.getX());
            SmartDashboard.putNumber("Robot Y (Meters)", robotFieldPosition.getY());
            SmartDashboard.putNumber("Robot Rotation (Deg)", robotFieldPosition.getRotation().getDegrees());
        }
    }
}