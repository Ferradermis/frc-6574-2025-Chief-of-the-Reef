package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PhoenixUtil;

// TODO: Unused for now until we get the parts to put Krakens on the elevator
public class ElevatorIOKraken implements ElevatorIO {

  public static final double sppolRadius = 0; //TODO: defaulted to 0 until I can look at the robot/CAD
  public PositionVoltage request;
  public TalonFX motor;

  public ElevatorIOKraken(int motorId) {
    motor = new TalonFX(motorId);
    request = new PositionVoltage(0);
    motor.setControl(request);
    configureTalons();
  }

  private void configureTalons() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Voltage.PeakForwardVoltage = 7; //TODO: Probably need to change this value
    config.Voltage.PeakReverseVoltage = 7; //TODO: Probably need to change this value
    config.CurrentLimits.StatorCurrentLimit = 0; //TODO: find value
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 0; //TODO: find value
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Slot0.kP = 1.0;
    PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
  }
  
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.distance.mut_replace(Meters.of(motor.getPosition().getValue().in(Degrees) * 2 * Math.PI * sppolRadius));
    inputs.velocity.mut_replace(MetersPerSecond.of(motor.getVelocity().getValue().in(DegreesPerSecond) * 2 * Math.PI * sppolRadius));
    inputs.setpoint.mut_replace(
        Distance.ofRelativeUnits(
            ((MotionMagicVoltage) motor.getAppliedControl()).Position, Meters));
    inputs.supplyCurrent.mut_replace(motor.getStatorCurrent().getValue());
  }

  @Override
  public void setTarget(Distance meters) {
    request = request.withPosition(meters.in(Meters)/(2 * Math.PI * sppolRadius));
    motor.setControl(request);
  }

  @Override
  public void stop() {
    motor.setControl(new StaticBrake());
  }
}
