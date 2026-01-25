package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RollerSubsystem;

public class Spindexer extends RollerSubsystem {
  public Spindexer(LoggedTalonFX motor) {
    super(
        motor.withConfig(LoggedTalonFX.buildStandardConfig(40, 40)),
        new LoggedTunableNumber("Spindexer/Speed", 0.5),
        new LoggedTunableNumber("Spindexer/UnjamSpeed", 0.25));
  }
}
