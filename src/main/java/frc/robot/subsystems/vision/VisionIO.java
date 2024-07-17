package frc.robot.subsystems.vision;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.math.geometry.Pose3d;

public interface VisionIO {

    @AutoLog
    public static class VisionIOInputs {
        public Pose3d sideCameraPose = new Pose3d();
        public Pose3d frontCameraPose = new Pose3d();
    }

    public default void updateInputs(VisionIOInputs inputs) {}
    
}
