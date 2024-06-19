package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// Subsystem base automatically runs periodic, too scared to put stuff in robot.java
public class Pivot extends SubsystemBase {

  private final PivotIOInputsAutoLogged inputs = new PivotIOInputsAutoLogged();
  SysIdRoutine sysId;
  PivotIO io;

  public Pivot(PivotIO io) {
    this.io = io;

    switch (Constants.currentMode) {
      case REAL:
        io.configurePID(1.0, 0, 0);
        break;
      case REPLAY:
        io.configurePID(1.0, 0.0, 0.0);
        break;
      case SIM:
        io.configurePID(1.5, 0.0, 0.1);
        break;
      default:
        break;
    }

    // Using SysID is wild
    sysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Pivot/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
  }

  private void runVolts(double volts) {
    io.setVoltage(volts);
  }

  // IN RADIANS!!!!!!!!!!!!!!!!!!!!!!!!!!
  public void setPosition(Double position) {
    io.setPosition(position);
  }

  public Command setPositionCommand(DoubleSupplier posRad) {
    return run(() -> setPosition(posRad.getAsDouble()));
  }

  public Command getDefaultCommand() {
    return setPositionCommand(() -> 0.0);
  }

  public boolean nearSetPoint() {
    return io.nearSetPoint();
  }

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    return sysId.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    return sysId.dynamic(direction);
  }

  @Override
  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Pivot", inputs);
    Logger.recordOutput(
        "Pivot/Running Command",
        Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));
  }
}
