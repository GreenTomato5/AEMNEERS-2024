package frc.robot.subsystems.Vision;

import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class Vision {
  public static enum CameraResolution {
    HIGH_RES,
    NORMAL
  }

  public static final Transform3d frontCamToRobot =
      new Transform3d(
          new Translation3d(Units.inchesToMeters(-14.25), 0, Units.inchesToMeters(6)),
          new Rotation3d(0, Units.degreesToRadians(-67), Units.degreesToRadians(180)));
}
