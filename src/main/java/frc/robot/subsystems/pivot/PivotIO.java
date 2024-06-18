package frc.robot.subsystems.pivot;

import org.littletonrobotics.junction.AutoLog;

public interface PivotIO {
    
    @AutoLog
    public static class PivotIOInputs {
        public double pivotCurrentPosition = 0.0;
        public double pivotAppliedVolts = 0.0;
        public double pivotSetpoint = 0.0;
    }

    /** Updates the set of loggable inputs. */
    public default void updateInputs(PivotIOInputs inputs) {}

    /** Sets the angle of the intake, in radians. */
    public default void setPosition(double positionRad) {}

    /** Run open loop at the specified voltage. */
    public default void setVoltage(double volts) {}

    /** Stop in open loop. */
    public default void stop() {}

    /** Set velocity PID constants. */
    public default void configurePID(double kP, double kI, double kD) {}
}
