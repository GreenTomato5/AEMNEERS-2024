package frc.robot.subsystems.spinner;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

// At this point im just putting override everywhere where it doesent make an error, I should lowkey
// google what it is

public class SpinnerIOSim implements SpinnerIO {
  // Took from other repo this is based on (idk what type of sim to use, y isnt this flywheel)
  private final DCMotorSim intakeSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.01);
  // IN RPM!!!
  public double speedPoint = 0.0;
  private double appliedVolts = 0.0;
  private boolean closedLoop = false;
  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

  public void updateInputs(SpinnerIOInputs inputs) {
    if (closedLoop) {
      // its divided by 60 because you cant get RPS in dc motor sim beacuse ???? idrk
      appliedVolts = pid.calculate(intakeSim.getAngularVelocityRPM() / 60, speedPoint);
      intakeSim.setInputVoltage(appliedVolts);
    }

    intakeSim.update(0.02);

    inputs.wheelSpeed = intakeSim.getAngularVelocityRPM();
    inputs.wheelSpeedPoint = speedPoint;
    inputs.wheelAppliedVolts = appliedVolts;
  }

  public void setSpeed(double speed) {
    closedLoop = true;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    intakeSim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    closedLoop = false;
    appliedVolts = 0.0;
    setVoltage(0);
  }

  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }
}
