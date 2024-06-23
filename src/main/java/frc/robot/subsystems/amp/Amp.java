package frc.robot.subsystems.amp;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import org.littletonrobotics.junction.Logger;

// Subsystem base automatically runs periodic, too scared to put stuff in robot.java
public class Amp extends SubsystemBase {

  private final AmpIOInputsAutoLogged inputs = new AmpIOInputsAutoLogged();

  private Translation3d zeroedTranslation3d;
  private Pose3d zeroedPose3d;

  SysIdRoutine spinnerSysId;
  SysIdRoutine pivotSysId;
  AmpIO io;

  public Amp(AmpIO io) {
    this.io = io;
    // TODO: Tune feedback controllers, these values are MADE UP
    switch (Constants.currentMode) {
      case REAL:
        io.configurePivotPID(1, 0, 0);
        io.configureSpinnerPID(1, 0, 0);
        break;
      case REPLAY:
        io.configurePivotPID(1, 0, 0);
        io.configurePivotPID(1, 0, 0);
        break;
      case SIM:
        io.configurePivotPID(3, 0, 0);
        io.configureSpinnerPID(1, 0, 0);
        break;
      default:
        break;
    }

    spinnerSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Amp/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism(
                (voltage) -> runSpinnerVolts(voltage.in(Volts)), null, this));

    pivotSysId =
        new SysIdRoutine(
            new SysIdRoutine.Config(
                null,
                null,
                null,
                (state) -> Logger.recordOutput("Amp/SysIdState", state.toString())),
            new SysIdRoutine.Mechanism((voltage) -> runPivotVolts(voltage.in(Volts)), null, this));
  }

  // Pivot Stuff
  private void runPivotVolts(double volts) {
    io.setPivotVoltage(volts);
  }

  public void setPosition(Double position) {
    io.setPivotPosition(position);
  }

  public void logPose3d() {
    zeroedTranslation3d = new Translation3d(-0.317, 0, 0.48);
    // Negative so it goes the right direction
    zeroedPose3d = new Pose3d(zeroedTranslation3d, new Rotation3d(0, -io.getPivotPosition(), 0));
    Logger.recordOutput("Amp/AmpBarPose3D", zeroedPose3d);
  }

  public Command setPositionCommand(DoubleSupplier posRad) {
    return run(() -> setPosition(posRad.getAsDouble()));
  }

  // Spinner Stuff

  private void runSpinnerVolts(double volts) {
    io.setSpinnerVoltage(volts);
  }

  public void setSpinnerSpeed(Double speed) {
    io.setSpinnerSpeed(speed);
  }

  public Command setSpeedCommand(DoubleSupplier rps) {
    return run(() -> setSpinnerSpeed(rps.getAsDouble()));
  }

  // Other
  public void setSpeedAndPosition(DoubleSupplier rps, DoubleSupplier radPos) {
    setSpinnerSpeed(rps.getAsDouble());
    setPosition(radPos.getAsDouble());
  }

  public Command ampCommand(DoubleSupplier rps, DoubleSupplier radPos) {
    return run(() -> setSpeedAndPosition(rps, radPos));
  }

  public Command getDefaultCommand() {
    return ampCommand(() -> 0.0, () -> 0.0);
  }

  @Override
  public void periodic() {
    logPose3d();
    io.updateInputs(inputs);
    Logger.processInputs("Amp", inputs);
    Logger.recordOutput(
        "Amp/Running Command",
        Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));
  }
}
