package frc.robot.subsystems.Vision;

import static frc.robot.Constants.aprilTagFieldLayout;
import static frc.robot.subsystems.Vision.Vision.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.math.geometry.*;
import java.util.Optional;
import org.photonvision.EstimatedRobotPose;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;
import org.photonvision.simulation.PhotonCameraSim;
import org.photonvision.simulation.SimCameraProperties;
import org.photonvision.simulation.VisionSystemSim;

public class VisionIOSim implements VisionIO {
  private final VisionSystemSim visionSim;

  // Forward Camera
  private final PhotonCameraSim frontCam;
  private final PhotonPoseEstimator frontPhotonPoseEstimator;

  private Pose3d[] poseArray = new Pose3d[3];
  private double[] timestampArray = new double[3];
  private double[] visionStdArray = new double[9];

  public VisionIOSim() {
    PhotonCamera front = new PhotonCamera("front");
    frontPhotonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, front, frontCamToRobot);

    visionSim = new VisionSystemSim("main");
    visionSim.addAprilTags(aprilTagFieldLayout);

    SimCameraProperties frontCameraProp = new SimCameraProperties();
    frontCameraProp.setCalibration(1600, 1200, Rotation2d.fromDegrees(95.95));
    frontCameraProp.setCalibError(0.25, 0.10);
    frontCameraProp.setFPS(25);
    frontCameraProp.setAvgLatencyMs(50);
    frontCameraProp.setLatencyStdDevMs(15);

    frontCam = new PhotonCameraSim(front, frontCameraProp);

    visionSim.addCamera(frontCam, frontCamToRobot);

    frontCam.enableDrawWireframe(true);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    getEstimatedPoseUpdates();
    inputs.visionPoses = poseArray;
    inputs.timestamps = timestampArray;
    inputs.visionStdDevs = visionStdArray;
  }

  public void updatePose(Pose2d pose) {
    visionSim.update(pose);
  }

  public void getEstimatedPoseUpdates() {
    Optional<EstimatedRobotPose> pose = frontPhotonPoseEstimator.update();
  }
}
