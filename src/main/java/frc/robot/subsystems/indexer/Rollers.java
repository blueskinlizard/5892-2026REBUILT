package frc.robot.subsystems.indexer;

import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RollerSubsystem;

public class Rollers extends RollerSubsystem {
  public Rollers(LoggedTalonFX motor) {
    super(
        motor.withConfig(LoggedTalonFX.buildStandardConfig(40, 40)),
        new LoggedTunableNumber("Rollers/Speed", 0.5),
        new LoggedTunableNumber("Rollers/UnjamSpeed", 0.25));
  }
}
