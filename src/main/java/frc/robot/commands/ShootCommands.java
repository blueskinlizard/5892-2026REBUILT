package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Flywheel;
import frc.robot.subsystems.shooter.Hood;
import frc.robot.subsystems.shooter.Turret;
import lombok.AccessLevel;
import lombok.NoArgsConstructor;

@NoArgsConstructor(access = AccessLevel.PRIVATE)
public class ShootCommands {
  public Command shoot(Indexer indexer, Flywheel flywheel, Hood hood, Turret turret) {
    return Commands.parallel(
        indexer.outtake(), flywheel.aimCommand(), hood.aimCommand(), turret.aimCommand());
  }
}
