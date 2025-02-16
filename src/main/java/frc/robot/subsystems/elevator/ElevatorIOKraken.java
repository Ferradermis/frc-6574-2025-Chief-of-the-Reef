package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOKraken implements ElevatorIO {

  public static final double spoolRadius = 2.312; //TODO: defaulted to 0 until I can look at the robot/CAD
  public MotionMagicVoltage request;
  public TalonFX leftMotor;
  public TalonFX rightMotor;

  // Create a new instance of the ElevatorIOKraken subsystem
  // Creates two new TalonFX motor controllers for the elevator and a new voltage output request
  public ElevatorIOKraken(int leftMotorId, int rightMotorId) {
    leftMotor = new TalonFX(leftMotorId);
    rightMotor = new TalonFX(rightMotorId);
    request = new MotionMagicVoltage(0);
    leftMotor.setControl(request);
    rightMotor.setControl(request);
    configureTalons();
  }

  // Configures the TalonFX motor controllers
  private void configureTalons() {
    TalonFXConfiguration leftConfig = new TalonFXConfiguration();
    TalonFXConfiguration rightConfig = new TalonFXConfiguration();
    leftConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    leftConfig.Voltage.PeakForwardVoltage = 1; //TODO: Probably need to change this value
    leftConfig.Voltage.PeakReverseVoltage = 1; //TODO: Probably need to change this value
    leftConfig.CurrentLimits.StatorCurrentLimit = 80; //TODO: find value
    leftConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    leftConfig.CurrentLimits.SupplyCurrentLimit = 40; //TODO: find value
    leftConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    leftConfig.Slot0.kP = 2.0;
    leftConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    rightConfig.Voltage.PeakForwardVoltage = 1; //TODO: Probably need to change this value
    rightConfig.Voltage.PeakReverseVoltage = 1; //TODO: Probably need to change this value
    rightConfig.CurrentLimits.StatorCurrentLimit = 80; //TODO: find value
    rightConfig.CurrentLimits.StatorCurrentLimitEnable = true;
    rightConfig.CurrentLimits.SupplyCurrentLimit = 40; //TODO: find value
    rightConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
    rightConfig.Slot0.kP = 2.0;
    rightConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    PhoenixUtil.tryUntilOk(5, () -> leftMotor.getConfigurator().apply(leftConfig));
    PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(leftConfig));
  }
  
  // Updates the inputs of the elevator
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.distance.mut_replace(Inches.of(leftMotor.getPosition().getValue().in(Rotations) * 2 * Math.PI * spoolRadius));
    inputs.rightDist.mut_replace(Meters.of(rightMotor.getPosition().getValue().in(Rotations) * 2 * Math.PI * spoolRadius));
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
