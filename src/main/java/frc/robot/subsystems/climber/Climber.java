package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {
  private final ClimberIOInputsAutoLogged inputs = new ClimberIOInputsAutoLogged();

  ClimberIO io;
  SysIdRoutine sysId;

  public Climber(ClimberIO io) {
    this.io = io;

    // Taken directly from pivot :)
    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(1.0, 0, 0);
        break;
      case REPLAY:
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        io.configurePID(3.0, 0.0, 0.1);
        break;
      default:
        break;
    }

    sysId = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,
            null,
            null,
            (state) -> Logger.recordOutput("Climber/SysIdState", state.toString())),
        new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  public Command setPositionCommand(DoubleSupplier leftPosRad, DoubleSupplier rightPosRad) {
    return run(() -> setPosition(leftPosRad.getAsDouble(), rightPosRad.getAsDouble()));
  }

  public Command getDefaultCommand() {
    return setPositionCommand(() -> 0.0, () -> 0.0);
  }

  public void runVolts(double volts) {
    io.setVoltage(volts);
  }

  public void setPosition(double leftPosRads, double rightPosRads) {
    io.setRotations(leftPosRads, rightPosRads);
  }

  public void periodic() {
    io.updateInputs(inputs);
  }
}
