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

public class ElevatorIOKraken implements ElevatorIO {

  public static final double spoolRadius = 0; //TODO: defaulted to 0 until I can look at the robot/CAD
  public PositionVoltage request;
  public TalonFX leftMotor;
  public TalonFX rightMotor;

  // Create a new instance of the ElevatorIOKraken subsystem
  // Creates two new TalonFX motor controllers for the elevator and a new voltage output request
  public ElevatorIOKraken(int leftMotorId, int rightMotorId) {
    leftMotor = new TalonFX(leftMotorId);
    rightMotor = new TalonFX(rightMotorId);
    request = new PositionVoltage(0);
    leftMotor.setControl(request);
    rightMotor.setControl(request);
    configureTalons();
  }

  // Configures the TalonFX motor controllers
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
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(config));
  }
  
  // Updates the inputs of the elevator
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.distance.mut_replace(Meters.of(leftMotor.getPosition().getValue().in(Degrees) * 2 * Math.PI * spoolRadius));
    inputs.velocity.mut_replace(MetersPerSecond.of(leftMotor.getVelocity().getValue().in(DegreesPerSecond) * 2 * Math.PI * spoolRadius));
    inputs.setpoint.mut_replace(
        Distance.ofRelativeUnits(
            ((MotionMagicVoltage) leftMotor.getAppliedControl()).Position, Meters));
    inputs.supplyCurrent.mut_replace(leftMotor.getStatorCurrent().getValue());
  }

  // Sets the target distance of the elevator
  @Override
  public void setTarget(Distance meters) {
    request = request.withPosition(meters.in(Meters)/(2 * Math.PI * spoolRadius));
    leftMotor.setControl(request);
    rightMotor.setControl(request);
  }

  // Stops the motors of the elevator
  @Override
  public void stop() {
    leftMotor.setControl(new StaticBrake());
    rightMotor.setControl(new StaticBrake());
  }
}
