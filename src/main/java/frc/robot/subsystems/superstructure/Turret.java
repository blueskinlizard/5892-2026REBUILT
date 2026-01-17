package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Rotations;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.configs.TorqueCurrentConfigs;

import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTalon.LoggedTalonFX;

public class Turret extends SubsystemBase {
    private final MutAngle turretAngle = Rotations.mutable(0);

    private final LoggedTalonFX motor;

    public Turret (LoggedTalonFX motor) {
        var config = new TalonFXConfiguration().withCurrentLimits(
            new CurrentLimitsConfigs().withStatorCurrentLimit(10)
        ).withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(15))
        .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0));
        this.motor = motor.withConfig(config).withMMPIDTuning(config);
    }
}
