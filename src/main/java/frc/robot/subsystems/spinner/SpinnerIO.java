package frc.robot.subsystems.spinner;

import org.littletonrobotics.junction.AutoLog;

public interface SpinnerIO {
        @AutoLog
        public static class SpinnerIOInputs {
            public double wheelSpeed = 0.0;
            public double wheelSpeedPoint = 0.0;
            public double wheelAppliedVolts = 0.0;
        }

        /** Updates the set of loggable inputs. */
        public default void updateInputs(SpinnerIOInputs inputs) {}

        /** Sets the angle of the intake, in radians. */
        public default void setSpeed(double speed) {}

        /** Run open loop at the specified voltage. */
        public default void setVoltage(double volts) {}

        /** Stop in open loop. */
        public default void stop() {}

        /** Set velocity PID constants. */
        public default void configurePID(double kP, double kI, double kD) {}        
}
