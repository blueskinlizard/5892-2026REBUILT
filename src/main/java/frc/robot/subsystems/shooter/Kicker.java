package frc.robot.subsystems.shooter;

import frc.robot.util.LoggedTalon.LoggedTalonFX;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.RollerSubsystem;

public class Kicker extends RollerSubsystem {
  public Kicker(LoggedTalonFX motor) {
    super(
        motor.withConfig(LoggedTalonFX.buildStandardConfig(40, 40)),
        new LoggedTunableNumber("Kicker/Speed", 1.00));
    this.setDefaultCommand(this.runRoller(Direction.FORWARD));
  }
}
