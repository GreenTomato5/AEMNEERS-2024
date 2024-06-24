package frc.robot.subsystems.shooter;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc.robot.Constants;

public class ShooterIOSim implements ShooterIO {

  // Uhhh so idk what the correct values for this is either other than the num
  // motors, someone
  // TODO: please get the values
  private FlywheelSim sim = new FlywheelSim(DCMotor.getFalcon500(2), 1.5, 0.004);
  private PIDController pid = new PIDController(0.0, 0.0, 0.0);
  private SimpleMotorFeedforward ff;

  private boolean closedLoop = false;
  private double ffVolts = 0.0;
  private double appliedVolts = 0.0;
  public double speedPoint = 0.0;

  @Override
  public void updateInputs(ShooterIOInputs inputs) {
    if (closedLoop) {
      appliedVolts =
          MathUtil.clamp(pid.calculate(sim.getAngularVelocityRadPerSec()) + ffVolts, -12.0, 12.0);
      sim.setInputVoltage(appliedVolts);
    }

    sim.update(0.02);

    inputs.shooterSpeed = sim.getAngularVelocityRPM() / 60;
    inputs.shooterAppliedVolts = appliedVolts;
    inputs.shooterSpeedPoint = speedPoint;
  }

  @Override
  public void setVoltage(double volts) {
    closedLoop = false;
    appliedVolts = volts;
    sim.setInputVoltage(volts);
  }

  @Override
  public void setSpeed(double rps) {
    closedLoop = true;
    speedPoint = rps;

    double feedbackVoltage = pid.calculate(sim.getAngularVelocityRPM() / 60, rps);
    double feedforwardVoltage = ff.calculate(rps);

    sim.setInputVoltage(feedbackVoltage + feedforwardVoltage);
  }

  @Override
  public void stop() {
    setVoltage(0.0);
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setPID(kP, kI, kD);
  }

  @Override
  public void configureFeedForward(double kS, double kV, double kA) {
    ff = new SimpleMotorFeedforward(kS, kV, kA);
  }

  public boolean nearSpeedPoint() {
    return Math.abs(speedPoint - sim.getAngularVelocityRPM() / 60) < Constants.Spinner.THRESHOLD;
  }
}
