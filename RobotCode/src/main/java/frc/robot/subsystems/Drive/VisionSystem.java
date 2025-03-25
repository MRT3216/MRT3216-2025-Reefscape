// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Drive;

import java.util.HashMap;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;
import frc.robot.settings.FieldConstants;
import frc.robot.settings.VisionConstants.CamerasConstants;

public class VisionSystem {
    // 
    private HashMap<String, Camera> cams = new HashMap<String, Camera>();
    private VisionSystemSim visionSim = new VisionSystemSim("Vision");

    public VisionSystem(CommandSwerveDrivetrain driveTrain) {
        if (Robot.isSimulation()) {
            visionSim = new VisionSystemSim("Vision");
            visionSim.addAprilTags(FieldConstants.fieldLayout);
        }

        // Add all cameras based on the constants
        for (CamerasConstants cameraConstants : CamerasConstants.values()) {
            Camera cam = new Camera(cameraConstants, driveTrain, visionSim);
            cams.put(cameraConstants.photonName, cam);
        }
    }

    /**
     * Adds all filtered poses to the drive train
     */
    public void addAllFilteredPoses() {
        for (Camera camera : cams.values()) {
            // If the camera is enabled, add the filtered pose
            if (camera.isEnabled().getAsBoolean()) {
                camera.addFilteredPose();
            } 

            SmartDashboard.putBoolean(camera.getPhotonName() + " Enabled", camera.isEnabled().getAsBoolean());
            SmartDashboard.putString(camera.getPhotonName() + " Strat", camera.getPrimaryStrategy().toString());
        }
    }

    /**
     * Updates the vision system to use the strategy during general driving
     */
    public void configureFarStrategy() {
        cams.get(CamerasConstants.BACK_LEFT.photonName).setEnabled(true);
        cams.get(CamerasConstants.BACK_RIGHT.photonName).setEnabled(true);

        cams.get(CamerasConstants.FRONT_LEFT.photonName).setStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        cams.get(CamerasConstants.FRONT_RIGHT.photonName).setStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
    }

    /**
     * Updates the vision system to use the strategy during close driving.
     * This is triggered by the CloseDriveToPose command
     */
    public void configureCloseStrategy() {
        cams.get(CamerasConstants.BACK_LEFT.photonName).setEnabled(false);
        cams.get(CamerasConstants.BACK_RIGHT.photonName).setEnabled(false);

        // cams.get(CamerasConstants.FRONT_LEFT.photonName).setStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        // cams.get(CamerasConstants.FRONT_RIGHT.photonName).setStrategy(PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR);
        cams.get(CamerasConstants.FRONT_LEFT.photonName).setStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
        cams.get(CamerasConstants.FRONT_RIGHT.photonName).setStrategy(PoseStrategy.PNP_DISTANCE_TRIG_SOLVE);
    }
}
