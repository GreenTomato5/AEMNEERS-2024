package frc.robot.subsystems.climber;
import org.littletonrobotics.junction.AutoLog;

public interface ClimberIO {
    @AutoLog
    public static class ClimberIOInputs {
        public double leftClimberSpeed = 0.0;
        public double rightClimberSpeed = 0.0;
        public double leftClimberSetpointRads = 0.0;
        public double rightClimberSetpointRads = 0.0;
        public double leftClimberAppliedVolts = 0.0;
        public double rightClimberAppliedVolts = 0.0;
        public double leftClimberMeasuredRads = 0.0;
        public double rightClimberMeasuredRads = 0.0;
    }

    public default void updateInputs(ClimberIOInputs inputs) {
    }

    public default void setRotations(double leftSetpoint, double rightSetpoint) {
    }

    public default void setVoltage(double volts) {
    }

    public default void stop() {
    }

    public default void configurePID(double kP, double kI, double kD) {
    }

    public default boolean leftClimberOverCurrentLimit(double currentLimit) {
        return false;
    }

    public default boolean rightClimberOverCurrentLimit(double currentLimit) {
        return false;
    }

}
