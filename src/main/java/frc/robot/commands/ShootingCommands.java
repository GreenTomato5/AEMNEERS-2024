package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.spinner.Spinner;

public class ShootingCommands {

  public static Command shootSpeaker(Shooter shooter, Spinner spinner) {
    return shooter
        .setSpeedCommand(() -> Constants.Shooter.ON)
        .until(() -> shooter.nearSpeedPoint())
        .andThen(
            shooter.setSpeedCommand(() -> Constants.Shooter.ON),
            spinner.setSpeedCommand(() -> Constants.Spinner.FEEDING));
  }

  public static Command intakeNote(Spinner spinner, Pivot pivot) {
    return spinner
        .setSpeedCommand(() -> Constants.Spinner.ON)
        .alongWith(pivot.setPositionCommand(() -> Constants.Pivot.OUT))
        .until(() -> pivot.nearSetPoint())
        .andThen(Commands.waitSeconds(1));
  }
}
