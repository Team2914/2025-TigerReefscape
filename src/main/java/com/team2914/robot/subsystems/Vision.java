package com.team2914.robot.subsystems;

import java.io.IOException;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;
import org.photonvision.targeting.PhotonTrackedTarget;

import com.team2914.robot.Robot;
import com.team2914.robot.Constants.VisionConstants;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Vision extends SubsystemBase {
    public static Vision instance;

    private VisionSystemSim visionSim;
    private SimCameraProperties frontCameraSimProp;
    private PhotonCamera frontCamera;
    private PhotonCameraSim frontCameraSim;


    PhotonPoseEstimator photonPoseEstimator;

    public static Vision getInstance() {
        if (instance == null) {
           return new Vision();
        }

        return instance;
    }

    private Vision() {

        AprilTagFieldLayout fieldLayout = AprilTagFieldLayout.loadField(AprilTagFields.k2025Reefscape);

        // Our camera is mounted 0.1 meters forward and 0.5 meters up from the robot pose,
        // (Robot pose is considered the center of rotation at the floor level, or Z = 0)
        Translation3d robotToCameraTrl = new Translation3d(-15, 0, 0.5); // 15 in ?? up
        // and pitched 15 degrees up.
        Rotation3d robotToCameraRot = new Rotation3d(0, Math.toRadians(-15), 0);
        Transform3d robotToCamera = new Transform3d(robotToCameraTrl, robotToCameraRot);

        frontCamera = new PhotonCamera(VisionConstants.frontCameraName);
        photonPoseEstimator = new PhotonPoseEstimator(fieldLayout, PoseStrategy.CLOSEST_TO_REFERENCE_POSE, robotToCamera);
        
        if (RobotBase.isSimulation()) {
            visionSim = new VisionSystemSim("main");

           

            visionSim.addAprilTags(fieldLayout);

            frontCameraSimProp = new SimCameraProperties();

            // A 640 x 480 camera with a 100 degree diagonal FOV.
            frontCameraSimProp.setCalibration(640, 480, Rotation2d.fromDegrees(100));
            // Approximate detection noise with average and standard deviation error in pixels.
            frontCameraSimProp.setCalibError(0.25, 0.08);
            // Set the camera image capture framerate (Note: this is limited by robot loop rate).
            frontCameraSimProp.setFPS(VisionConstants.camFPS);
            // The average and standard deviation in milliseconds of image data latency.
            frontCameraSimProp.setAvgLatencyMs(VisionConstants.avrgMs);

            frontCameraSim = new PhotonCameraSim(frontCamera, frontCameraSimProp);

            // Add this camera to the vision system simulation with the given robot-to-camera transform.
            visionSim.addCamera(frontCameraSim, robotToCamera);
        }
    }

    @Override
    public void simulationPeriodic(){
        visionSim.update(Drivetrain.getInstance().getPose());


        visionSim.getDebugField();
    }

    public PhotonTrackedTarget getBestTarget() {
        if(Robot.isSimulation()){
            return frontCameraSim.getCamera().getLatestResult().getBestTarget();
        }

        return frontCamera.getLatestResult().getBestTarget();
    }

    public boolean hasTargets(){
        return Robot.isSimulation() ? !frontCameraSim.getCamera().getLatestResult().getTargets().isEmpty() : !frontCamera.getLatestResult().getTargets().isEmpty();
    }
}