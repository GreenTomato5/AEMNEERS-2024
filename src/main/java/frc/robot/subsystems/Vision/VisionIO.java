package frc.robot.subsystems.Vision;


import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import java.util.List;
import org.littletonrobotics.junction.AutoLog;

public interface VisionIO {
  @AutoLog
  public static class AprilTagVisionIOInputs {
    public Pose3d[] visionPoses =
        List.of(new Pose3d(), new Pose3d(), new Pose3d()).toArray(new Pose3d[0]);
    public double[] timestamps = new double[3];
    public double[] latency = new double[3];
    public double[] visionStdDevs = new double[9];
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AprilTagVisionIOInputs inputs) {}

  /** Update the reference pose of the vision system. Currently only used in sim. */
  public default void updatePose(Pose2d pose) {}

  /**
   * The standard deviations of the estimated poses from vision cameras, for use with {@link
   * edu.wpi.first.math.estimator.SwerveDrivePoseEstimator SwerveDrivePoseEstimator}.
   *
   * @param estimatedPose The estimated pose to guess standard deviations for. /*
   */
}
