package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOKraken implements ElevatorIO {

  public static final double spoolRadius = 2.312/2;
  public MotionMagicVoltage request;
  public TalonFX followerMotor;
  public TalonFX leaderMotor;
  private Distance setpoint = Inches.of(0);

  // Create a new instance of the ElevatorIOKraken subsystem
  // Creates two new TalonFX motor controllers for the elevator and a new voltage output request
  public ElevatorIOKraken(int leftMotorId, int rightMotorId) {
    followerMotor = new TalonFX(leftMotorId);
    leaderMotor = new TalonFX(rightMotorId);
    request = new MotionMagicVoltage(0);
    leaderMotor.setPosition(0);
    followerMotor.setPosition(0);
    configureKrakens();
  }

  // Configures the TalonFX motor controllers
  private void configureKrakens() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.Voltage.PeakForwardVoltage = 11; //TODO: Probably need to change this value
    config.Voltage.PeakReverseVoltage = -11; //TODO: Probably need to change this value
    config.CurrentLimits.StatorCurrentLimit = 80;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Slot0.kP = 4;
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.3;
    config.Slot0.kV = 0.0;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.Feedback.SensorToMechanismRatio = 1/25;

    TalonFXConfiguration config2 = new TalonFXConfiguration();
    config2.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config2.Voltage.PeakForwardVoltage = 11; //TODO: Probably need to change this value
    config2.Voltage.PeakReverseVoltage = -11; //TODO: Probably need to change this value
    config2.CurrentLimits.StatorCurrentLimit = 80;
    config2.CurrentLimits.StatorCurrentLimitEnable = true;
    config2.CurrentLimits.SupplyCurrentLimit = 40;
    config2.CurrentLimits.SupplyCurrentLimitEnable = true;
    config2.Slot0.kP = 4;  
    config2.Slot0.kG = 0.0;
    config2.Slot0.kS = 0.3;
    config2.Slot0.kV = 0.0;
    config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config2.Feedback.SensorToMechanismRatio = 1/25;

    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config2));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 100.0;
    motionMagicConfigs.MotionMagicAcceleration = 200;
    motionMagicConfigs.MotionMagicJerk = 0.0;
    motionMagicConfigs.MotionMagicExpo_kV = 0.0;
    motionMagicConfigs.MotionMagicExpo_kA = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(motionMagicConfigs));
    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(motionMagicConfigs));
  }
  
  // Updates the inputs of the elevator
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.distance.mut_replace(Inches.of(followerMotor.getPosition().getValue().in(Rotations) * 2 * Math.PI * spoolRadius));
    inputs.rightDist.mut_replace(Inches.of(leaderMotor.getPosition().getValue().in(Rotations) * 2 * Math.PI * spoolRadius));
    inputs.velocity.mut_replace(InchesPerSecond.of(followerMotor.getVelocity().getValue().in(RotationsPerSecond) * 2 * Math.PI * spoolRadius));
    inputs.setpoint.mut_replace(setpoint);
    inputs.supplyCurrent.mut_replace(leaderMotor.getSupplyCurrent().getValue());
    inputs.statorCurrent.mut_replace(leaderMotor.getStatorCurrent().getValue());
    inputs.voltageSetpoint.mut_replace(leaderMotor.getMotorVoltage().getValue());
  }

  @Override
  public void reset() {
    leaderMotor.setPosition(0);
    followerMotor.setPosition(0);
  }

  // Sets the target distance of the elevator
  @Override
  public void setTarget(Distance target) {
    request = request.withPosition(target.in(Inches)/(2 * Math.PI * spoolRadius));
    leaderMotor.setControl(request);
    followerMotor.setControl(request);
    setpoint = target;
  }

  @Override
  public void setVoltage(double voltage) {
    leaderMotor.setVoltage(voltage);
    followerMotor.setVoltage(voltage);
  }

  // Stops the motors of the elevator
  @Override
  public void stop() {
    followerMotor.setControl(new StaticBrake());
    leaderMotor.setControl(new StaticBrake());
  }
}
