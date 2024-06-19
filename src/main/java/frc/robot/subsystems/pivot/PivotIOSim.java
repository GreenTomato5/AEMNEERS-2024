package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {
  // I guessed innacuratley, idk what half of this means fr
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getNEO(1), 100, SingleJointedArmSim.estimateMOI(0.7, 2.0), 0.5, 0, 20, true, 0.0);

  private final PIDController pid = new PIDController(0.0, 0.0, 0.0);

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  // IN RADIANTS! (I think)
  public double setPoint = 0.0;

  @Override
  public void updateInputs(PivotIOInputs inputs) {
    if (closedLoop) {
      // Need this bc of L sysID that uses open loop
      appliedVolts = pid.calculate(sim.getAngleRads(), setPoint);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.pivotCurrentPosition = sim.getAngleRads();
    inputs.pivotAppliedVolts = appliedVolts;
    inputs.pivotSetpoint = setPoint;
  }

  @Override
  public void setPosition(double positionRad) {
    closedLoop = true;
    setVoltage(pid.calculate(sim.getAngleRads(), setPoint));
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  public boolean nearSetPoint() {
    return Math.abs(setPoint - sim.getAngleRads()) < Constants.Spinner.THRESHOLD;
  }
}
