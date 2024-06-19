package frc.robot.subsystems.pivot;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class PivotIOSim implements PivotIO {
  // I guessed innacuratley, idk what half of this means fr
  private final SingleJointedArmSim sim =
      new SingleJointedArmSim(
          DCMotor.getKrakenX60(1),
          67.5, // gearing
          0.192383865, // MOI
          0.3, // arm length
          Units.degreesToRadians(0), // min angle -- floor
          Units.degreesToRadians(180), // max angle -- hard stop
          false,
          Units.degreesToRadians(0));
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
    inputs.pivotVelocity = sim.getVelocityRadPerSec();
  }

  @Override
  public void setPosition(double positionRad) {
    closedLoop = true;
    setPoint = positionRad;
    double feedbackVoltage = pid.calculate(sim.getAngleRads(), positionRad);
    appliedVolts = feedbackVoltage;

    sim.setInputVoltage(feedbackVoltage);
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

  public double getPivotPosition() {
    return sim.getAngleRads();
  }

  public boolean nearSetPoint() {
    return Math.abs(setPoint - sim.getAngleRads()) < Constants.Spinner.THRESHOLD;
  }
}
