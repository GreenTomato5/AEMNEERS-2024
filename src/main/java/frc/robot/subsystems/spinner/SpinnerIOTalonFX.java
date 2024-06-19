package frc.robot.subsystems.spinner;

import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.controller.PIDController;

public class SpinnerIOTalonFX implements SpinnerIO {

  private final TalonFX motor;
  public double speedPoint = 0.0;
  private PIDController controller;

  public SpinnerIOTalonFX() {
    motor = new TalonFX(1);
    controller = new PIDController(0, 0, 0);
  }

  @Override
  public void updateInputs(SpinnerIOInputs inputs) {
    inputs.wheelSpeed = motor.get();
    inputs.wheelAppliedVolts = motor.getMotorVoltage().getValueAsDouble();
    inputs.wheelSpeedPoint = speedPoint;
  }

  public void setSpeed(Double speed) {
    motor.setVoltage(controller.calculate(motor.getVelocity().getValueAsDouble(), speedPoint));
  }

  @Override
  public void stop() {
    motor.stopMotor();
  }

  public void setVoltage(Double volts) {
    motor.setVoltage(speedPoint);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }
}
