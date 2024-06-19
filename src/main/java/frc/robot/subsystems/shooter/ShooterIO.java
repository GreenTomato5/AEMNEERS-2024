package frc.robot.subsystems.shooter;

import org.littletonrobotics.junction.AutoLog;

public interface ShooterIO {
  @AutoLog
  public static class ShooterIOInputs {
    public double rightShooterSpeed = 0.0;
    public double leftShooterSpeed = 0.0;
    public double leftShooterAppliedVolts = 0.0;
    public double rightShooterAppliedVolts = 0.0;
    public double shooterSpeedPoint = 0.0;

    public double shooterSpeed = 0.0;
    public double shooterAppliedVolts = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets the angle of the intake, in radians. */
  public default void setSpeed(double rps) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}

  /** Set ff constants */
  public default void configureFeedForward(double kS, double kV, double kA) {}

  public default boolean nearSpeedPoint() {
    return false;
  }
}
