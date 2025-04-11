package frc.robot.subsystems.endEffector;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class EndEffectorIOKraken implements EndEffectorIO {
  public VoltageOut request;
  public TalonFX motor;

  // Create a new instance of the EndEffectorIOKraken subsystem
  // Creates a new TalonFX motor controller for the end effector and a new voltage output request
  public EndEffectorIOKraken(int motorId) {
    motor = new TalonFX(motorId);
    request = new VoltageOut(0.0);

    motor.setControl(request);
    configureTalons();
  }

  // Configures the TalonFX motor controller
  private void configureTalons() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = 40; // Coral EE: 20
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40; // Coral EE: 20
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }

  // Updates the inputs of the EndEffectorIOKraken subsystem
  @Override
  public void updateInputs(EndEffectorInputs inputs) {
    inputs.angularVelocity.mut_replace(motor.getRotorVelocity().getValue());
    inputs.velocity = motor.getRotorVelocity().getValueAsDouble();
    inputs.voltageSetpoint.mut_replace(
        Voltage.ofRelativeUnits(((VoltageOut) motor.getAppliedControl()).Output, Volts));
    inputs.voltage.mut_replace(motor.getMotorVoltage().getValue());
    inputs.supplyCurrent.mut_replace(motor.getStatorCurrent().getValue());
  }

  // Sets the target voltage of the end effector
  @Override
  public void setTarget(Voltage target) {
    request = request.withOutput(target);
    motor.setControl(request);
  }

  // Stops the motor of the end effector
  @Override
  public void stop() {
    motor.setControl(new StaticBrake());
  }
}
