package frc.robot.subsystems.superstructure;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.LoggedTalonFX;

public class Hood extends SubsystemBase {

  private final LoggedTalonFX hoodMotor;
  private final LoggedDIO limitSwitch;

  private final MotionMagicVoltage mmControl = new MotionMagicVoltage(0);

  public Hood(LoggedTalonFX hoodMotor, LoggedDIO limitSwitch) {
    var config =
        new TalonFXConfiguration()
            .withSlot0(new Slot0Configs().withKP(0).withKI(0).withKD(0).withKS(0).withKV(0))
            .withMotionMagic(
                new MotionMagicConfigs()
                    .withMotionMagicCruiseVelocity(15)
                    .withMotionMagicAcceleration(30))
            .withMotorOutput(
                new MotorOutputConfigs()
                    .withNeutralMode(NeutralModeValue.Brake)
                    .withInverted(InvertedValue.Clockwise_Positive))
            .withCurrentLimits(new CurrentLimitsConfigs().withStatorCurrentLimit(20))
            .withFeedback(new FeedbackConfigs().withSensorToMechanismRatio(15));
    this.hoodMotor = hoodMotor.withConfig(config).withMMPIDTuning(config);
    this.limitSwitch = limitSwitch.withReversed(true);
  }

  // TODO: Add homing

  @Override
  public void periodic() {
    hoodMotor.periodic();
    limitSwitch.periodic();
  }
}
