package frc.robot.subsystems.pivot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;

public class PivotIOSparkMax implements PivotIO {

    private final CANSparkMax motor;
    private final RelativeEncoder encoder;
    private final SparkPIDController pidController;
    private double setPoint = 0.0;

    public PivotIOSparkMax(int deviceID) {
        motor = new CANSparkMax(deviceID, MotorType.kBrushless);
        encoder = motor.getEncoder();
        pidController = motor.getPIDController();
        motor.restoreFactoryDefaults();
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
        pidController.setReference(setPoint, CANSparkMax.ControlType.kPosition);
    }

    @Override
    public void setVoltage(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void stop() {
        motor.stopMotor();
    }

    @Override
    public void configurePID(double kP, double kI, double kD) {
        pidController.setP(kP);
        pidController.setI(kI);
        pidController.setD(kD);
    }
}
