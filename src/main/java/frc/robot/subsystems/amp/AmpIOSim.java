package frc.robot.subsystems.amp;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.Constants;

public class AmpIOSim implements AmpIO {
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

  private FlywheelSim spinnerSim = new FlywheelSim(DCMotor.getFalcon500(2), 1.5, 0.004);
  private PIDController spinnerPid = new PIDController(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward ff;

  private boolean closedLoop = false;
  private double appliedVolts = 0.0;
  public double setPoint = 0.0;

  private boolean closedSpinnerLoop = false;
  private double ffVolts = 0.0;
  private double appliedSpinnerVolts = 0.0;
  public double speedPoint = 0.0;

  @Override
  public void updateInputs(AmpIOInputs inputs) {
    if (closedLoop) {
      // Need this bc of L sysID that uses open loop
      appliedVolts = pid.calculate(sim.getAngleRads(), setPoint);
      sim.setInputVoltage(appliedVolts);
    }

    if (closedSpinnerLoop) {
      appliedVolts =
          MathUtil.clamp(
              spinnerPid.calculate(spinnerSim.getAngularVelocityRadPerSec()) + ffVolts,
              -12.0,
              12.0);
      spinnerSim.setInputVoltage(appliedSpinnerVolts);
    }

    sim.update(0.02);

    inputs.ampShooterVelocity = spinnerSim.getAngularVelocityRPM() / 60;
    inputs.ampShooterVoltage = appliedSpinnerVolts;
    inputs.ampShooterSpeedPoint = speedPoint;

    sim.update(0.02);

    inputs.ampBarCurrentPosition = sim.getAngleRads();
    inputs.ampBarAppliedVolts = appliedVolts;
    inputs.ampBarSetpoint = setPoint;
    inputs.ampBarVelocity = sim.getVelocityRadPerSec();
  }

  @Override
  public void setSpinnerVoltage(double volts) {
    closedSpinnerLoop = false;
    appliedSpinnerVolts = volts;
    spinnerSim.setInputVoltage(volts);
  }

  @Override
  public void setSpeed(double rps) {
    closedSpinnerLoop = true;
    speedPoint = rps;

    double feedbackVoltage = spinnerPid.calculate(spinnerSim.getAngularVelocityRPM() / 60, rps);
    double feedforwardVoltage = ff.calculate(rps);

    spinnerSim.setInputVoltage(feedbackVoltage + feedforwardVoltage);
  }

  @Override
  public void stopSpinner() {
    setSpinnerVoltage(0.0);
  }

  @Override
  public void configureSpinnerPID(double kP, double kI, double kD) {
    spinnerPid.setPID(kP, kI, kD);
  }

  @Override
  public void configureFeedForward(double kS, double kV, double kA) {
    ff = new SimpleMotorFeedforward(kS, kV, kA);
  }

  public boolean nearSpeedPoint() {
    return Math.abs(speedPoint - spinnerSim.getAngularVelocityRPM() / 60)
        < Constants.Spinner.THRESHOLD;
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
