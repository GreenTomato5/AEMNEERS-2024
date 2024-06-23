package frc.robot.subsystems.amp;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;

public class AmpIOReal implements AmpIO {
    
    public double setPoint = 0.0;
    public double speedPoint = 0.0;

    private final CANSparkMax leftPivot;
    private final CANSparkMax rightPivot;
    private final TalonFX spinnerMotor;

    private RelativeEncoder leftEncoder;
    private RelativeEncoder rightEncoder;

    private PIDController spinnerController;
    private PIDController pivotController;

    public AmpIOReal() {
        leftPivot = new CANSparkMax(1, MotorType.kBrushless);
        rightPivot = new CANSparkMax(0, MotorType.kBrushless);
        spinnerMotor = new TalonFX(8);

        leftEncoder = leftPivot.getEncoder();
        rightEncoder = rightPivot.getEncoder();

        leftPivot.setInverted(false);
        rightPivot.setInverted(true);

        pivotController = new PIDController(0, 0, 0);
        spinnerController = new PIDController(0, 0, 0);

        leftEncoder.setPositionConversionFactor(2 * Math.PI);
        rightEncoder.setPositionConversionFactor(2 * Math.PI);
    }

    @Override
    public void updateInputs(AmpIOInputs inputs) {
        inputs.leftPivotCurrentPosition = leftEncoder.getPosition();
        inputs.rightPivotCurrentPosition = rightEncoder.getPosition();
        inputs.leftPivotAppliedVolts = leftPivot.getAppliedOutput() * leftPivot.getBusVoltage();
        inputs.rightPivotAppliedVolts = rightPivot.getAppliedOutput() * rightPivot.getBusVoltage();
        inputs.leftPivotSetpoint = setPoint;
        inputs.rightPivotSetpoint = setPoint;

        inputs.spinnerAppliedVolts = speedPoint;
        inputs.spinnerSpeedPoint = spinnerMotor.getMotorVoltage().getValueAsDouble();
        inputs.spinnerVelocity = spinnerMotor.getVelocity().getValueAsDouble();

    }

    // Pivot Stuff
    public void setPivotPosition(double positionRad) {
        setPoint = positionRad;
        leftPivot.set(pivotController.calculate(leftEncoder.getPosition(), positionRad));
        rightPivot.set(pivotController.calculate(rightEncoder.getPosition(), positionRad));
    }

    public void setPivotVoltage(double volts) {
        leftPivot.setVoltage(volts);
        rightPivot.setVoltage(volts);
    }
  
    public void stopPivot() {
        leftPivot.stopMotor();
        rightPivot.stopMotor();
    }
  
    public void configurePivotPID(double kP, double kI, double kD) {
        pivotController.setPID(kP, kI, kD);
    }
  
    //Spinner Stuff

    public void setSpinnerSpeed(double rps) {
        speedPoint = rps;
        spinnerMotor.set(spinnerController.calculate(spinnerMotor.getRotorVelocity().getValueAsDouble(), rps));
    }

    public void setSpinnerVoltage(double volts) {
        spinnerMotor.setVoltage(volts);
    }
  
    public void stopSpinner() {
        spinnerMotor.stopMotor();
    }
  
    public void configureSpinnerPID(double kP, double kI, double kD) {
        spinnerController.setPID(kP, kI, kD);
    }
}