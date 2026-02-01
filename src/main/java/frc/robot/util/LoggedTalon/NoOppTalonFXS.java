package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.SlotConfigs;
import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.controls.ControlRequest;
import edu.wpi.first.units.measure.Angle;
import java.util.function.Function;

public class NoOppTalonFXS extends LoggedTalonFXS {

  public NoOppTalonFXS(String name, int followers) {
    super(name, followers);
  }

  @Override
  public void setControl(ControlRequest controlRequest) {}

  @Override
  protected void updateInputs(TalonInputs inputs) {}

  @Override
  public LoggedTalonFXS withConfig(TalonFXSConfiguration config) {
    return this;
  }

  @Override
  public LoggedTalonFXS withSimConfig(
      Function<TalonFXSConfiguration, TalonFXSConfiguration> config) {
    return this;
  }

  @Override
  public void quickApplyConfig(TalonFXSConfiguration config) {}

  @Override
  public void quickApplyConfig(SlotConfigs config) {}

  @Override
  public void quickApplyConfig(MotionMagicConfigs config) {}

  @Override
  public void setPosition(Angle position) {}
}
