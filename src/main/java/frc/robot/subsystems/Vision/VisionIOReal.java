package frc.robot.subsystems.Vision;

import static frc.robot.Constants.aprilTagFieldLayout;
import static frc.robot.subsystems.Vision.Vision.*;
import static org.photonvision.PhotonPoseEstimator.PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR;

import edu.wpi.first.math.geometry.Pose3d;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonPoseEstimator;

public class VisionIOReal implements VisionIO {
  // Forward Camera
  private final PhotonCamera frontCam;
  private final PhotonPoseEstimator frontPhotonPoseEstimator;

  private Pose3d[] poseArray = new Pose3d[3];
  private double[] timestampArray = new double[3];
  private double[] visionStdArray = new double[9];
  private double[] latencyArray = new double[3];
  private int count = 0;

  public VisionIOReal() {
    frontCam = new PhotonCamera("front");
    frontPhotonPoseEstimator =
        new PhotonPoseEstimator(
            aprilTagFieldLayout, MULTI_TAG_PNP_ON_COPROCESSOR, frontCam, frontCamToRobot);
  }

  @Override
  public void updateInputs(AprilTagVisionIOInputs inputs) {
    inputs.visionPoses = poseArray;
    inputs.timestamps = timestampArray;
    inputs.visionStdDevs = visionStdArray;
    inputs.latency = latencyArray;
    count += 1;
    if (count % 500 == 0) {
      frontCam.takeOutputSnapshot();
    }
  }
}
