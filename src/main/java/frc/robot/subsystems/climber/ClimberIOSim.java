package frc.robot.subsystems.climber;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import frc.robot.subsystems.climber.ClimberIO.ClimberIOInputs;

public class ClimberIOSim implements ClimberIO {
  private final DCMotorSim climberSim = new DCMotorSim(DCMotor.getFalcon500(1), 1, 0.01);

  private double leftSetpoint = 0.0;
  private double rightSetpoint = 0.0;
  private double appliedVolts = 0.0;

  private PIDController controller = new PIDController(0, 0, 0);

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberAppliedVolts = appliedVolts;
    inputs.rightClimberAppliedVolts = appliedVolts;
    inputs.leftClimberSpeed = climberSim.getAngularVelocityRadPerSec();
    inputs.rightClimberSpeed = climberSim.getAngularVelocityRadPerSec();
    inputs.leftClimberSetpointRads = leftSetpoint;
    inputs.rightClimberSetpointRads = rightSetpoint;
    inputs.leftClimberMeasuredRads = climberSim.getAngularPositionRad();
    inputs.rightClimberMeasuredRads = climberSim.getAngularPositionRad();
  }

  public void setRotations(double leftSetpoint, double rightSetpoint) {
    // zzz ima just use one of the two for both because im not doing allat with 2 sims
    appliedVolts = controller.calculate(climberSim.getAngularPositionRad(), leftSetpoint);
    climberSim.setInputVoltage(appliedVolts);
  }

  public void setVoltage(double volts) {
    appliedVolts = volts;
    climberSim.setInputVoltage(volts);
  }

  public void stop() {
    climberSim.setInputVoltage(0);
  }

  public void configurePID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }

  // Not doing allat or using this
  public boolean leftClimberOverCurrentLimit(double currentLimit) {
    return false;
  }

  public boolean rightClimberOverCurrentLimit(double currentLimit) {
    return false;
  }
}
