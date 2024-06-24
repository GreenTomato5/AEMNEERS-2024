package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;

public class ClimberIOSparkMax implements ClimberIO {

  private CANSparkMax leftClimber;
  private CANSparkMax rightClimber;

  private RelativeEncoder leftEncoder;
  private RelativeEncoder rightEncoder;

  private double leftSetpoint = 0.0;
  private double rightSetpoint = 0.0;

  private PIDController controller = new PIDController(0, 0, 0);

  public ClimberIOSparkMax() {
    leftClimber = new CANSparkMax(0, MotorType.kBrushless);
    rightClimber = new CANSparkMax(0, MotorType.kBrushless);
    leftClimber.setInverted(false);
    rightClimber.setInverted(true);
    leftEncoder = leftClimber.getEncoder();
    rightEncoder = rightClimber.getEncoder();

    leftEncoder.setPositionConversionFactor(2 * Math.PI);
    rightEncoder.setPositionConversionFactor(2 * Math.PI);
  }

  public void updateInputs(ClimberIOInputs inputs) {
    inputs.leftClimberSpeed = leftEncoder.getVelocity() / 60;
    inputs.rightClimberSpeed = rightEncoder.getVelocity() / 60;
    inputs.leftClimberSetpointRads = leftSetpoint;
    inputs.rightClimberSetpointRads = rightSetpoint;
    inputs.leftClimberAppliedVolts = leftClimber.getAppliedOutput() * leftClimber.getBusVoltage();
    inputs.rightClimberAppliedVolts = rightClimber.getAppliedOutput() * leftClimber.getBusVoltage();
    inputs.leftClimberMeasuredRads = leftEncoder.getPosition();
    inputs.rightClimberMeasuredRads = leftEncoder.getPosition();
  }

  public void setRotations(double leftSetpoint, double rightSetpoint) {
    this.leftSetpoint = leftSetpoint;
    this.rightSetpoint = rightSetpoint;
    leftClimber.set(controller.calculate(leftEncoder.getPosition(), leftSetpoint));
    rightClimber.set(controller.calculate(rightEncoder.getPosition(), rightSetpoint));
  }

  public void setVoltage(double volts) {
    leftClimber.setVoltage(volts);
    rightClimber.setVoltage(volts);
  }

  public void stop() {
    leftClimber.stopMotor();
    rightClimber.stopMotor();
  }

  public void configurePID(double kP, double kI, double kD) {
    controller.setPID(kP, kI, kD);
  }

  // Im not going to use these bc idk how to implement them in sim so womp womp
  // zzzz

  public boolean leftClimberOverCurrentLimit(double currentLimit) {
    return leftClimber.getOutputCurrent() > currentLimit;
  }

  public boolean rightClimberOverCurrentLimit(double currentLimit) {
    return rightClimber.getOutputCurrent() > currentLimit;
  }
}
