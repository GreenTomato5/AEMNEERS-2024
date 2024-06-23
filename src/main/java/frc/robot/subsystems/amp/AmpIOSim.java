package frc.robot.subsystems.amp;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class AmpIOSim implements AmpIO {
  // Literally just the intake values LOL
  private final SingleJointedArmSim pivotSim =
      new SingleJointedArmSim(
          DCMotor.getNEO(2),
          67.5,
          0.192383865,
          0.3,
          Units.degreesToRadians(0),
          Units.degreesToRadians(180),
          false,
          Units.degreesToRadians(0));
  // These values are more wrong than you can imagine
  private FlywheelSim spinnerSim = new FlywheelSim(DCMotor.getKrakenX60(1), 1.5, 0.004);

  private PIDController pivotController = new PIDController(0.0, 0.0, 0.0);
  private PIDController spinnerController = new PIDController(0.0, 0.0, 0.0);

  private double pivotAppliedVolts = 0.0;
  private double spinnerAppliedVolts = 0.0;

  public double setPoint = 0.0;
  public double speedPoint = 0.0;

  public void updateInputs(AmpIOInputs inputs) {
    inputs.leftPivotCurrentPosition = pivotSim.getAngleRads();
    inputs.rightPivotCurrentPosition = pivotSim.getAngleRads();
    inputs.leftPivotAppliedVolts = pivotAppliedVolts;
    inputs.rightPivotAppliedVolts = pivotAppliedVolts;
    inputs.leftPivotSetpoint = setPoint;
    inputs.rightPivotSetpoint = setPoint;

    inputs.spinnerAppliedVolts = spinnerAppliedVolts;
    inputs.spinnerSpeedPoint = speedPoint;
    inputs.spinnerVelocity = spinnerSim.getAngularVelocityRPM() / 60;

    pivotSim.update(0.02);
    spinnerSim.update(0.02);
  }

  // Pivot Stuff

  public void setPivotPosition(double positionRad) {
    pivotAppliedVolts = pivotController.calculate(pivotSim.getAngleRads(), positionRad);
    pivotSim.setInputVoltage(pivotAppliedVolts);
  }

  public double getPivotPosition() {
    return pivotSim.getAngleRads();
  }

  public void setPivotVoltage(double volts) {
    pivotAppliedVolts = 0;
    pivotSim.setInputVoltage(volts);
  }

  public void stopPivot() {
    pivotAppliedVolts = 0;
    pivotSim.setInputVoltage(0);
  }

  public void configurePivotPID(double kP, double kI, double kD) {
    pivotController.setPID(kP, kI, kD);
  }

  // Spinner Stuff
  public void setSpinnerSpeed(double rps) {
    spinnerAppliedVolts = spinnerController.calculate(spinnerSim.getAngularVelocityRPM() / 60, rps);
    spinnerSim.setInputVoltage(spinnerAppliedVolts);
  }

  public void setSpinnerVoltage(double volts) {
    spinnerAppliedVolts = volts;
    spinnerSim.setInputVoltage(volts);
  }

  public void stopSpinner() {
    spinnerAppliedVolts = 0;
    spinnerSim.setInputVoltage(0);
  }

  public void configureSpinnerPID(double kP, double kI, double kD) {
    spinnerController.setPID(kP, kI, kD);
  }
}
