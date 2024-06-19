package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants;

public class PivotIOSparkMax implements PivotIO {

  private final CANSparkMax motor;
  private final RelativeEncoder encoder;
  private final PIDController pidController;
  public double setPoint = 0.0;

  public PivotIOSparkMax() {
    motor = new CANSparkMax(1, MotorType.kBrushless);
    encoder = motor.getEncoder();
    pidController = new PIDController(0, 0, 0);
    motor.restoreFactoryDefaults();
    encoder.setPositionConversionFactor(2 * Math.PI);
  }

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    inputs.pivotCurrentPosition = encoder.getPosition();
    inputs.pivotAppliedVolts = motor.getAppliedOutput() * motor.getBusVoltage();
    inputs.pivotSetpoint = setPoint;
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
