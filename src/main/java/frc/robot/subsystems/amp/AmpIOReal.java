package frc.robot.subsystems.amp;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class AmpIOReal implements AmpIO {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final PIDController pidController;
    public double setPoint = 0.0;
    private final TalonFX wheelsMotor = new TalonFX(8);
    private PIDController feedbackController;
    private SimpleMotorFeedforward feedForwardController;
    public double speedPoint = 0.0;

    public AmpIOReal() {
        motor = new CANSparkMax(1, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pidController = new PIDController(0, 0, 0);
        motor.restoreFactoryDefaults();
        encoder.setPositionConversionFactor(2 * Math.PI);
        wheelsMotor.setInverted(false);
        feedbackController = new PIDController(0, 0, 0);
    }

    @Override
    public void updateInputs(AmpIOInputs inputs) {
        inputs.ampBarCurrentPosition = encoder.getPosition();
        inputs.ampBarAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
        inputs.ampBarSetpoint = setPoint;
        inputs.ampBarVelocity = motor.getEncoder().getVelocity();
        inputs.ampShooterSpeedPoint = speedPoint;
        inputs.ampBarAppliedVolts = wheelsMotor.getMotorVoltage().getValueAsDouble();
        inputs.ampShooterVelocity = wheelsMotor.getVelocity().getValueAsDouble();

    }

    public void setSpeed(double rps) {
        speedPoint = rps;
        double feedforward = feedForwardController.calculate(rps);
        double volts = feedbackController.calculate(wheelsMotor.getRotorVelocity().getValueAsDouble(), rps)
                + feedforward;

        wheelsMotor.setVoltage(volts);
    }

    public void setSpinnerVoltage(double volts) {
        wheelsMotor.setVoltage(volts);
    }

    public void stopSpinner() {
        wheelsMotor.stopMotor();
    }

    public void configureSpinnerPID(double kP, double kI, double kD) {
        feedbackController.setPID(kP, kI, kD);
    }

    public void configureFeedForward(double kS, double kV, double kA) {
        feedForwardController = new SimpleMotorFeedforward(kS, kV, kA);
    }

    public boolean speedPoint() {
        return Math.abs(speedPoint - wheelsMotor.getVelocity().getValueAsDouble()) < Constants.Spinner.THRESHOLD;
    }

    @Override
    public void setPosition(double positionRad) {
        setPoint = positionRad;
        motor.setVoltage(pidController.calculate(encoder.getPosition(), setPoint));
    }

    @Override
    public void setVoltage(double volts) {
        // Open loop for sysID
        motor.setVoltage(volts);
    }

    public double getPivotPosition() {
        return encoder.getPosition();
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pidController.setPID(kP, kI, kD);
    }

    public boolean nearSetPoint() {
        return Math.abs(setPoint - encoder.getPosition()) < Constants.Pivot.THRESHOLD;
    }
}