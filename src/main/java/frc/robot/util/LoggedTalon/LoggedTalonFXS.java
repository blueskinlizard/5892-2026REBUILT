package frc.robot.util.LoggedTalon;

import com.ctre.phoenix6.configs.TalonFXSConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import java.util.function.Function;

public abstract class LoggedTalonFXS extends LoggedTalon {

  public LoggedTalonFXS(String name, int followers) {
    super(name, followers);
  }

  /**
   * Build a standard config with a few commonly adjusted values
   *
   * @param statorCurrentLimit Stator current limit in amps This limit directly affects max torque
   * @param supplyCurrentLimit Supply current limit in amps This limit affects brownouts and battery
   *     usage
   * @param invert Motor direction. Default: CCW positive
   * @param neutralMode Neutral mode. Default: Coast
   * @return the built config
   */
  public static TalonFXSConfiguration buildStandardConfig(
      double statorCurrentLimit,
      double supplyCurrentLimit,
      InvertedValue invert,
      NeutralModeValue neutralMode) {
    var config = new TalonFXSConfiguration();
    config.CurrentLimits.StatorCurrentLimit = statorCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimit = supplyCurrentLimit;
    config.MotorOutput.Inverted = invert;
    config.MotorOutput.NeutralMode = neutralMode;
    return config;
  }

  /**
   * Build a standard config with a few commonly adjusted values The NeutralMode is Coast
   *
   * @param statorCurrentLimit Stator current limit in amps This limit directly affects max torque
   * @param supplyCurrentLimit Supply current limit in amps This limit affects brownouts and battery
   *     usage
   * @param invert Motor direction. Default: CCW positive
   * @return the built config
   */
  public static TalonFXSConfiguration buildStandardConfig(
      double statorCurrentLimit, double supplyCurrentLimit, InvertedValue invert) {
    return LoggedTalonFXS.buildStandardConfig(
        statorCurrentLimit, supplyCurrentLimit, invert, NeutralModeValue.Coast);
  }

  /**
   * Build a standard config with a few commonly adjusted values The InvertValue is CCW positive
   *
   * @param statorCurrentLimit Stator current limit in amps This limit directly affects max torque
   * @param supplyCurrentLimit Supply current limit in amps This limit affects brownouts and battery
   *     usage
   * @param neutralMode Neutral mode. Default: Coast
   * @return the built config
   */
  public static TalonFXSConfiguration buildStandardConfig(
      double statorCurrentLimit, double supplyCurrentLimit, NeutralModeValue neutralMode) {
    return LoggedTalonFXS.buildStandardConfig(
        statorCurrentLimit,
        supplyCurrentLimit,
        InvertedValue.CounterClockwise_Positive,
        neutralMode);
  }

  public static TalonFXSConfiguration buildStandardConfig(
      double statorCurrentLimit, double supplyCurrentLimit) {
    return LoggedTalonFXS.buildStandardConfig(
        statorCurrentLimit,
        supplyCurrentLimit,
        InvertedValue.CounterClockwise_Positive,
        NeutralModeValue.Coast);
  }

  /**
   * Apply a config until it succeeds.
   *
   * @param config the config to apply
   * @return this for daisy-chaining
   */
  public abstract LoggedTalonFXS withConfig(TalonFXSConfiguration config);

  /**
   * Apply a config for simulation. This is ignored by a real IO interface. This should be called
   * after a normal config is applied.
   *
   * @param config Function to generate a config. This will only be called in simulation. Takes in
   *     the current config, modifies it for simulation, and returns it. This is meant to make the
   *     simulation resemble reality as much as possible.
   * @return This for daisy-chaining
   */
  public abstract LoggedTalonFXS withSimConfig(
      Function<TalonFXSConfiguration, TalonFXSConfiguration> config);

  public abstract void quickApplyConfig(TalonFXSConfiguration config);
}
