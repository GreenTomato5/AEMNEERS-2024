package frc.robot.subsystems.amp;

import org.littletonrobotics.junction.AutoLog;

import frc.robot.subsystems.shooter.ShooterIO.ShooterIOInputs;

public interface AmpIO {

  @AutoLog
  public static class AmpIOInputs {
    public double ampBarCurrentPosition = 0.0;
    public double ampBarAppliedVolts = 0.0;
    public double ampBarSetpoint = 0.0;
    public double ampBarVelocity = 0.0;
    public double ampShooterVelocity = 0.0;
    public double ampShooterSpeedPoint = 0.0;
    public double ampShooterVoltage = 0.0;



  }/** Updates the set of loggable inputs. */
  public default void updateInputs(AmpIOInputs inputs) {}

  /** Sets the angle of the amp bar, in radians. */
  public default void setPosition(double positionRad) {}

  /** Run open loop at the specified voltage. */
  public default void setVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stop() {}

  /** Set velocity PID constants. */
  public default void configurePID(double kP, double kI, double kD) {}

  public default boolean nearSetPoint() {
    return false;
  }

  public default double getAmpBarPosition() {
    return 0.0;
  }

    public default void updateInputs(ShooterIOInputs inputs) {}

  /** Sets the angle of the intake, in radians. */
  public default void setSpeed(double rps) {}

  /** Run open loop at the specified voltage. */
  public default void setSpinnerVoltage(double volts) {}

  /** Stop in open loop. */
  public default void stopSpinner() {}

  /** Set velocity PID constants. */
  public default void configureSpinnerPID(double kP, double kI, double kD) {}

  /** Set ff constants */
  public default void configureFeedForward(double kS, double kV, double kA) {}

  public default boolean nearSpeedPoint() {
    return false;
  }
}
