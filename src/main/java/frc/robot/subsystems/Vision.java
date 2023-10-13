// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;

import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.SimVisionSystem;
// NOTE: code tested against updated PV, but wait for release
// import org.photonvision.simulation.PhotonCameraSim;
// import org.photonvision.simulation.VisionSystemSim;
// import org.photonvision.estimation.VisionEstimation;
import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import frc.robot.Constants;

public class Vision {
    // variable to turn on/off our private tag layout
    // if this is false, the compiler should remove all the unused code.
    public static final boolean USE_PRIVATE_TAG_LAYOUT = false;
    
    // Use the multitag pose estimator
    public static final boolean USE_MULTITAG = true;

    // Plot vision solutions
    public static final boolean PLOT_TAG_SOLUTIONS = true;
    
    // constants for extra tags in the shed  (lengths in meters!!)
    static final double SHED_TAG_NODE_XOFFSET = 0.45;
    static final double SHED_TAG_NODE_ZOFFSET = 0.31;
    static final double SHED_TAG_SUBSTATION_YOFFSET = 1.19;

    // // Values for the Shed in late January
    // private static final AprilTagFieldLayout SHED_TAG_FIELD_LAYOUT = new AprilTagFieldLayout(new ArrayList<AprilTag>() {
    //     {
    //         add(constructTag(26, 0, 1.636, 0.865, 0));
    //         add(constructTag(25, 0, 3.24, 0.895, 0));
    //         add(constructTag(24, 1.915, 0, 0.857, 90));
    //         add(constructTag(23, 4.958, 0, 0.845, 90));
    //         add(constructTag(22, 7.763, 0, 0.896, 90));
    //         add(constructTag(21, 8.780, 1.373, 0.895, 180));
    //         add(constructTag(20, 8.780, 2.392, 0.946, 180));
    //     }
    // }, Constants.CUSTOM_FIELD_LENGTH, Constants.CUSTOM_FIELD_WIDTH);

    // Simulation support
    // private VisionSystemSim m_visionSim = null;  // future version
    private SimVisionSystem m_aprilTagCamSim = null;
    private static final double CAM_FOV_DEG = 100.0;
    private static final int CAM_RES_X = 800;
    private static final int CAM_RES_Y = 600;

    public static final String CAMERA_NAME = "ApriltagCamera";
    private final PhotonCamera m_aprilTagCamera = new PhotonCamera(CAMERA_NAME);
    private AprilTagFieldLayout m_aprilTagFieldLayout;

    // Forward B&W camera for Apriltags
    // relative position of the camera on the robot ot the robot center
    private final Transform3d m_robotToAprilTagCam = new Transform3d(
            new Translation3d(Units.inchesToMeters(3.5), -0.136, Units.inchesToMeters(24.75)),
            new Rotation3d(0.0, 0.0, 0.0));

    private final PhotonPoseEstimator m_photonPoseEstimator;

    public Vision() {
        try {
            m_aprilTagFieldLayout = AprilTagFieldLayout
                        .loadFromResource(AprilTagFields.k2023ChargedUp.m_resourceFile);
        } catch (IOException e) {
            System.out.println("Unable to load AprilTag layout " + e.getMessage());
            m_aprilTagFieldLayout = null;
        }

        if (USE_PRIVATE_TAG_LAYOUT && m_aprilTagFieldLayout != null) {
            System.out.println("Vision is currently using supplemented SHED tag layout");
            List<AprilTag> tags = m_aprilTagFieldLayout.getTags();
            // Red nodes
            tags.add(constructTagRelative(21, m_aprilTagFieldLayout.getTagPose(1).get(), SHED_TAG_NODE_XOFFSET, 0, SHED_TAG_NODE_ZOFFSET));
            tags.add(constructTagRelative(22, m_aprilTagFieldLayout.getTagPose(2).get(), SHED_TAG_NODE_XOFFSET, 0, SHED_TAG_NODE_ZOFFSET));
            tags.add(constructTagRelative(23, m_aprilTagFieldLayout.getTagPose(3).get(), SHED_TAG_NODE_XOFFSET, 0, SHED_TAG_NODE_ZOFFSET));
            // Red substation
            tags.add(constructTagRelative(24, m_aprilTagFieldLayout.getTagPose(4).get(), 0, SHED_TAG_SUBSTATION_YOFFSET, 0));
            // Blue substation
            tags.add(constructTagRelative(25, m_aprilTagFieldLayout.getTagPose(5).get(), 0, -SHED_TAG_SUBSTATION_YOFFSET, 0));
            // Blue nodes
            tags.add(constructTagRelative(26, m_aprilTagFieldLayout.getTagPose(6).get(), -SHED_TAG_NODE_XOFFSET, 0, SHED_TAG_NODE_ZOFFSET));
            tags.add(constructTagRelative(27, m_aprilTagFieldLayout.getTagPose(7).get(), -SHED_TAG_NODE_XOFFSET, 0, SHED_TAG_NODE_ZOFFSET));
            tags.add(constructTagRelative(28, m_aprilTagFieldLayout.getTagPose(8).get(), -SHED_TAG_NODE_XOFFSET, 0, SHED_TAG_NODE_ZOFFSET));

            m_aprilTagFieldLayout = new AprilTagFieldLayout(tags, Constants.CUSTOM_FIELD_LENGTH, Constants.CUSTOM_FIELD_WIDTH);
        }

        if (Constants.SIMULATION_SUPPORT) {
            // initialize a simulated camera. Must be done after creating the tag layout
            initializeSimulation();
        }

        if (USE_MULTITAG) {
            m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout, PoseStrategy.MULTI_TAG_PNP,
                    m_aprilTagCamera, m_robotToAprilTagCam);
            m_photonPoseEstimator.setMultiTagFallbackStrategy(PoseStrategy.CLOSEST_TO_REFERENCE_POSE);
        } else {
            m_photonPoseEstimator = new PhotonPoseEstimator(m_aprilTagFieldLayout,
                    PoseStrategy.CLOSEST_TO_REFERENCE_POSE, m_aprilTagCamera, m_robotToAprilTagCam);
        }

        // set the driver mode to false
        m_aprilTagCamera.setDriverMode(false);
    }

    public void updateSimulation(Pose2d pose) {
        // m_visionSim.update(pose);   // future version
        m_aprilTagCamSim.processFrame(pose);
    }

    public void updateOdometry(SwerveDrivePoseEstimator odometry, Field2d field) {
        if (!m_aprilTagCamera.isConnected())
            return;

        PhotonPipelineResult targetResult = m_aprilTagCamera.getLatestResult();

        SmartDashboard.putBoolean("vision/hasTargets", targetResult.hasTargets());
        if (!targetResult.hasTargets()) {
            // if no target, clean out the numbers
            SmartDashboard.putNumber("vision/targetID", -1);
            // SmartDashboard.putNumber("vision/tagOffsetX", 0);
            // SmartDashboard.putNumber("vision/tagOffsetY", 0);
            // SmartDashboard.putNumber("vision/tagOffsetYaw", 0);

            if (PLOT_TAG_SOLUTIONS) {
                clearTagSolutions(field);
            }    
            return;
        } else {
            // For debug: get the current best target.
            PhotonTrackedTarget target = targetResult.getBestTarget();
            SmartDashboard.putNumber("vision/targetID", target.getFiducialId());
            // Transform3d cameraToTarget = target.getBestCameraToTarget();
            // SmartDashboard.putNumber("vision/tagOffsetX", Units.metersToInches(cameraToTarget.getX()));
            // SmartDashboard.putNumber("vision/tagOffsetY", Units.metersToInches(cameraToTarget.getY()));
            // SmartDashboard.putNumber("vision/tagOffsetYaw", Math.toDegrees(cameraToTarget.getRotation().getZ()));
        }

        if (m_aprilTagFieldLayout == null)
            return;

        if (PLOT_TAG_SOLUTIONS) {
            plotTagSolutions(field, targetResult);
        }    
    
        // Estimate the robot pose.
        // If successful, update the odometry using the timestamp of the measurement
        Optional<EstimatedRobotPose> result = getEstimatedGlobalPose(odometry.getEstimatedPosition());
        SmartDashboard.putBoolean("vision/foundSolution", result.isPresent());
        if (result.isPresent()) {
            EstimatedRobotPose camPose = result.get();
            Pose2d estimatedPose = camPose.estimatedPose.toPose2d();

            double curImageTimeStamp = targetResult.getTimestampSeconds();
            odometry.addVisionMeasurement(estimatedPose, curImageTimeStamp);

            if (PLOT_TAG_SOLUTIONS) {
                plotPose(field, "visionPose", estimatedPose);

                // // NOTE this the new PV version as for Sept 2023
                // var pnpResults = VisionEstimation.estimateCamPosePNP(m_aprilTagCamera.getCameraMatrix().get(),
                //         m_aprilTagCamera.getDistCoeffs().get(), targetResult.getTargets(), m_aprilTagFieldLayout);
                // Pose3d alt = new Pose3d().plus(pnpResults.alt).plus(m_robotToAprilTagCam.inverse());
                // plotPose(field, "visionAltPose", alt.toPose2d());
            }
        }
        else if (PLOT_TAG_SOLUTIONS) {
            plotPose(field, "visionPose", null);
        }
    }

    private Optional<EstimatedRobotPose> getEstimatedGlobalPose(Pose2d prevEstimatedRobotPose) {
        try {
            m_photonPoseEstimator.setReferencePose(prevEstimatedRobotPose);
            return m_photonPoseEstimator.update();
        } catch (Exception e) {
            // bad! log this and keep going
            DriverStation.reportError("Exception running PhotonPoseEstimator", e.getStackTrace());
            return Optional.empty();
        }
    }

    // get the tag ID closest to vertical center of camera
    // we might want to use this to do fine adjustments on field element locations
    public int getCentralTagId(boolean wantSubstationTarget) {
        // make sure camera connected
        if (!m_aprilTagCamera.isConnected())
            return -1;

        var targetResult = m_aprilTagCamera.getLatestResult();
        // if (!targetResult.hasTargets()) {
        //     return -1;
        // }

        // make a temp holder var for least Y translation, set to first tags translation
        double minY = 1.0e6; // big number
        int targetID = -1;
        for (PhotonTrackedTarget tag : targetResult.getTargets()) { // for every target in camera            
            // find id for current tag we are focusing on
            int tempTagID = tag.getFiducialId();

            // if tag has an invalid ID then skip this tag
            if (tempTagID < 1 || tempTagID > 8) {
                continue;
            }

            boolean isSubstation = tempTagID == 5 || tempTagID == 4;

            // if aiming for substation
            if (wantSubstationTarget) {
                if (!isSubstation) { //exit if tags are grids
                    continue; //continue the for loop
                }
            } else { //if aiming for aiming for grid/not substation
                if (isSubstation) { // exit if tag is substation
                    continue;
                }
            }

            // get transformation to target
            Transform3d tagTransform = tag.getBestCameraToTarget();
            // get abs translation to target from transformation
            double tagY = Math.abs(tagTransform.getY());

            // looking for smallest absolute relative to camera Y
            // if abs Y translation of new tag is less then holder tag, it becomes holder tag
            if (tagY < minY) {
                minY = tagY;
                targetID = tempTagID; // remember targetID
            }
        }
        
        return targetID;
    }

    // get the pose for a tag.
    // will return null if the tag is not in the field map (eg -1)
    public Optional<Pose2d> getTagPose(int tagId) {
        // optional in case no target is found
        Optional<Pose3d> tagPose = m_aprilTagFieldLayout.getTagPose(tagId);
        if (tagPose.isEmpty()) {
            return Optional.empty(); //returns an empty optional
        }
        return Optional.of(tagPose.get().toPose2d());
    }

    private static AprilTag constructTag(int id, double x, double y, double z, double angle) {
        return new AprilTag(id, new Pose3d(x, y, z, new Rotation3d(0, 0, Math.toRadians(angle))));
    }

    // add a new tag relative to another tag. Assume the orientation is the same
    private static AprilTag constructTagRelative(int id, Pose3d basePose, double x, double y, double z) {
        return new AprilTag(id, new Pose3d(basePose.getX() + x, basePose.getY() + y, basePose.getZ() + z, basePose.getRotation()));
    }

    private void initializeSimulation() {
        // m_visionSim = new VisionSystemSim("LigerVision");
        // // for now, cheat on the specs of the camera
        // PhotonCameraSim cam = new PhotonCameraSim(m_aprilTagCamera);
        // m_visionSim.addCamera(cam, m_robotToAprilTagCam);

        // m_visionSim.addVisionTargets(m_aprilTagFieldLayout);
        
        m_aprilTagCamSim = new SimVisionSystem(CAMERA_NAME, CAM_FOV_DEG, m_robotToAprilTagCam, 10.0, CAM_RES_X,
                CAM_RES_Y, 10.0);

        m_aprilTagCamSim.addVisionTargets(m_aprilTagFieldLayout);
    }

    // --- Routines to plot the vision solutions on a Field2d   ---------

    private void clearTagSolutions(Field2d field) {
        if (field == null) return;
        field.getObject("tagSolutions").setPoses();
        field.getObject("visionPose").setPoses();        
        // field.getObject("visionAltPose").setPoses();        
    }

    private void plotTagSolutions(Field2d field, PhotonPipelineResult result) {
        if (field == null) return;

        List<Pose2d> poses = new ArrayList<Pose2d>();
        for (PhotonTrackedTarget target : result.getTargets()) {
            int targetFiducialId = target.getFiducialId();
            if (targetFiducialId == -1) continue;
            
            Optional<Pose3d> targetPosition = m_aprilTagFieldLayout.getTagPose(targetFiducialId);
            if (targetPosition.isEmpty()) continue;

            Pose3d pose3d = targetPosition.get().transformBy(target.getAlternateCameraToTarget().inverse())
                            .transformBy(m_robotToAprilTagCam.inverse());
            poses.add(pose3d.toPose2d());
            
            pose3d = targetPosition.get().transformBy(target.getBestCameraToTarget().inverse())
                            .transformBy(m_robotToAprilTagCam.inverse());
            poses.add(pose3d.toPose2d());
        }

        field.getObject("tagSolutions").setPoses(poses);
    }

    private void plotPose(Field2d field, String label, Pose2d pose) {
        if (field == null) return;
        if (pose == null) 
            field.getObject(label).setPoses();
        else
            field.getObject(label).setPose(pose);
    }
}