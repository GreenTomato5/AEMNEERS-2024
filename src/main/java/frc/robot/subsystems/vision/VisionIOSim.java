package frc.robot.subsystems.vision;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.VisionSystemSim;

import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import frc.robot.Constants;

public class VisionIOSim implements VisionIO {
    private final VisionSystemSim visionSim;

    private final PhotonCamera frontCamera;
    private final PhotonPoseEstimator frontPoseEstimator;

    private final PhotonCamera sideCamera;
    private final PhotonPoseEstimator sidePoseEstimator;

    public VisionIOSim() {
        visionSim = new VisionSystemSim("Main");

        frontCamera = new PhotonCamera( "front");
        frontPoseEstimator = new PhotonPoseEstimator(
            Constants.Vision.aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, Constants.Vision.robotToFrontCam);

        sideCamera = new PhotonCamera("side");
        sidePoseEstimator = new PhotonPoseEstimator(
            Constants.Vision.aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, frontCamera, Constants.Vision.robotToFrontCam);
        visionSim.addAprilTags(Constants.Vision.aprilTagFieldLayout);

    }
}
