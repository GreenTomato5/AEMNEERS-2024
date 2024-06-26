package frc.robot.subsystems.amp;

import org.littletonrobotics.junction.AutoLog;

public interface AmpIO {

  @AutoLog
  public static class AmpIOInputs {
    public double rightPivotCurrentPosition = 0.0;
    public double rightPivotAppliedVolts = 0.0;
    public double rightPivotSetpoint = 0.0;
    public double leftPivotCurrentPosition = 0.0;
    public double leftPivotAppliedVolts = 0.0;
    public double leftPivotSetpoint = 0.0;
    public double spinnerVelocity = 0.0;
    public double spinnerSpeedPoint = 0.0;
    public double spinnerAppliedVolts = 0.0;
  }

  /** Updates the set of loggable inputs. */
  public default void updateInputs(AmpIOInputs inputs) {}

  // Pivot Stuff
  public default void setPivotPosition(double positionRad) {}

  public default void setPivotVoltage(double volts) {}

  public default void stopPivot() {}

  public default void configurePivotPID(double kP, double kI, double kD) {}

  public default double getPivotPosition() {
    return 0.0;
  }

  // Bar Spinner Stuff
  public default void setSpinnerSpeed(double rps) {}

  public default void setSpinnerVoltage(double volts) {}

  public default void stopSpinner() {}

  public default void configureSpinnerPID(double kP, double kI, double kD) {}
}
