package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import com.ctre.phoenix6.hardware.TalonFXS;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Function;

/**
 * NOOP implementation of a {@link TalonFXS}
 *
 * <p>This is designed to be used in replay when the logger is replaying all inputs
 *
 * @see LoggedTalon
 * @see LoggedTalonFXS
 * @see NoOppTalonFX
 */
public class NoOppTalonFXS extends LoggedTalonFXS {

  /**
   * Create a TalonFXS that does nothing.
   *
   * <p>This is design for replay purposes where the logger will supply inputs.
   *
   * @param name The name of this instance. This <strong>MUST</strong> match the real and sim
   *     implementations.
   * @param followers The number of followers. This <strong>MUST</strong> match the real and sim
   *     implementations.
   */
  public NoOppTalonFXS(String name, int followers) {
    super(name, followers);
  }

  /** {@inheritDoc} */
  @Override
  public void setControl(ControlRequest controlRequest) {}

  /** {@inheritDoc} */
  @Override
  protected void updateInputs(TalonInputs inputs) {}

  /** {@inheritDoc} */
  @Override
  public void quickApplyConfig(SlotConfigs config) {}

  /** {@inheritDoc} */
  @Override
  public void quickApplyConfig(MotionMagicConfigs config) {}

  /** {@inheritDoc} */
  @Override
  public void setPosition(Angle position) {}

  /** {@inheritDoc} */
  @Override
  public LoggedTalonFXS withConfig(TalonFXSConfiguration config) {
    return this;
  }

  /** {@inheritDoc} */
  @Override
  public LoggedTalonFXS withSimConfig(
      Function<TalonFXSConfiguration, TalonFXSConfiguration> config) {
    return this;
  }

  /** {@inheritDoc} */
  @Override
  public void quickApplyConfig(TalonFXSConfiguration config) {}
}
