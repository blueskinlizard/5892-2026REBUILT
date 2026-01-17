package frc.robot.subsystems.superstructure;

import static edu.wpi.first.units.Units.Rotations;

import java.util.function.Supplier;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.FeedbackConfigs;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.NeutralOut;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.units.MutableMeasure;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.MutAngle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.LoggedTunableMeasure;
import frc.robot.util.LoggedTunableNumber;
import frc.robot.util.MechanismUtil;
import frc.robot.util.LoggedDIO.LoggedDIO;
import frc.robot.util.LoggedTalon.LoggedTalonFX;

public class Hood extends SubsystemBase { 

  private final LoggedTalonFX hoodMotor;
  private final LoggedDIO limitSwitch;

  private final LoggedTunableNumber homingVoltage = new LoggedTunableNumber("Hood/Homing/Voltage", 4,"v"); 
  private final LoggedTunableNumber homingConfirmationVoltage = new LoggedTunableNumber("Hood/Homing/ConfirmVoltage", 4,"v"); 
  private final LoggedTunableMeasure<MutAngle> homingSwitchPosition = new LoggedTunableMeasure<>("Hood/Homing/homePosition", Rotations.mutable(0)); 
  private final LoggedTunableMeasure<MutAngle> homingConfirmPosition = new LoggedTunableMeasure<>("Hood/Homing/homePosition", Rotations.mutable(0.1)); 

  private final MotionMagicVoltage mmControl = new MotionMagicVoltage(homingSwitchPosition.get());
  private final NeutralOut neutralControl = new NeutralOut();
  private final MutAngle targetPosition = Rotations.mutable(0);

  private boolean positionControl = false;

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

  public Command homingCommand() {
    return MechanismUtil.buildHomingCommand(hoodMotor, limitSwitch, this, homingVoltage, false, homingSwitchPosition, homingConfirmationVoltage, homingConfirmPosition)
    .beforeStarting(()->positionControl = false);
  }
  public Command requestPosition(Supplier<MutAngle> position) {
    return runOnce(() -> {
      targetPosition.mut_replace(position.get());
      positionControl = true;
      setControl();
    });
  }

  @Override
  public void periodic() {
    hoodMotor.periodic();
    limitSwitch.periodic();
    setControl();
  }

  private void setControl() {
    if (positionControl) {
      hoodMotor.setControl(mmControl.withPosition(targetPosition).withLimitReverseMotion(limitSwitch.get()).withLimitForwardMotion(false));
    } else {
      hoodMotor.setControl(neutralControl);
    }
  }
}
