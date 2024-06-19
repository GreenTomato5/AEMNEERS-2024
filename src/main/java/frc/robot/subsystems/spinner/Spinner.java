package frc.robot.subsystems.spinner;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;

import static edu.wpi.first.units.Units.*;

import java.util.Optional;
import java.util.function.DoubleSupplier;
public class Spinner extends SubsystemBase {
   
    private SpinnerIO io;
    private SpinnerIOInputsAutoLogged inputs = new SpinnerIOInputsAutoLogged();
    private SysIdRoutine sysId;

    public Spinner(SpinnerIO io) {
        this.io = io;

        // Nah tune these PID values bc i just took them from pivot
        switch (Constants.currentMode) {
            case REAL:
                io.configurePID(1.0, 0, 0);
                break;
            case REPLAY:
                io.configurePID(1.0, 0.0, 0.0);
                break;
            case SIM:
                io.configurePID(0.5, 0.0, 0.0);
                break;
            default:
                break;
        }

        sysId =
            new SysIdRoutine(
                new SysIdRoutine.Config(
                    null,
                    null,
                    null,
                    (state) -> Logger.recordOutput("Spinner/SysIdState", state.toString())),
                new SysIdRoutine.Mechanism((voltage) -> runVolts(voltage.in(Volts)), null, this));
    }

    public void setSpeed(Double rps) {
        io.setSpeed(rps);
    }

    public void runVolts(Double volts) {
        io.setVoltage(volts);
    }

    public Command setSpeedCommand(DoubleSupplier rps) {
        return run(() -> setSpeed(rps.getAsDouble()));
    }

    public Command getDefaultCommand() {
        return setSpeedCommand(() -> 0.0);
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Pivot", inputs);
        Logger.recordOutput(
        "Spinner/Running Command",
            Optional.ofNullable(this.getCurrentCommand()).map(Command::getName).orElse("None"));
    }
}
