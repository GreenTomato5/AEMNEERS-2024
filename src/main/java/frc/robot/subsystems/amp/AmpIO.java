package frc.robot.subsystems.amp;

import org.littletonrobotics.junction.AutoLog;

public interface AmpIO {

  @AutoLog
  public static class AmpIOInputs {
    public double pivotCurrentPosition = 0.0;
    public double pivotAppliedVolts = 0.0;
    public double pivotSetpoint = 0.0;
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

  // Bar Spinner Stuff
  public default void setSpinnerSpeed(double rps) {}

  public default void setSpinnerVoltage(double volts) {}

  public default void stopSpinner() {}

  public default void configureSpinnerPID(double kP, double kI, double kD) {}
}
