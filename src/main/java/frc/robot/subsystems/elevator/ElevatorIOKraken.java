package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Distance;
import frc.robot.util.PhoenixUtil;

public class ElevatorIOKraken implements ElevatorIO {

  public static final double spoolRadius = 2.312; //TODO: defaulted to 0 until I can look at the robot/CAD
  public MotionMagicVoltage request;
  public TalonFX followerMotor;
  public TalonFX leaderMotor;
  private Distance setpoint = Distance.ofBaseUnits(0, Inches);

  // Create a new instance of the ElevatorIOKraken subsystem
  // Creates two new TalonFX motor controllers for the elevator and a new voltage output request
  public ElevatorIOKraken(int leftMotorId, int rightMotorId) {
    followerMotor = new TalonFX(leftMotorId);
    leaderMotor = new TalonFX(rightMotorId);
    request = new MotionMagicVoltage(0);
    configureTalons();
  }

  // Configures the TalonFX motor controllers
  private void configureTalons() {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Voltage.PeakForwardVoltage = 1; //TODO: Probably need to change this value
    config.Voltage.PeakReverseVoltage = 1; //TODO: Probably need to change this value
    config.CurrentLimits.StatorCurrentLimit = 80; //TODO: find value
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = 40; //TODO: find value
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Slot0.kP = 0.0;
    config.Slot0.kG = 0.0;
    config.Slot0.kS = 0.0;
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    TalonFXConfiguration config2 = new TalonFXConfiguration();
    config2.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config2.Voltage.PeakForwardVoltage = 1; //TODO: Probably need to change this value
    config2.Voltage.PeakReverseVoltage = 1; //TODO: Probably need to change this value
    config2.CurrentLimits.StatorCurrentLimit = 80; //TODO: find value
    config2.CurrentLimits.StatorCurrentLimitEnable = true;
    config2.CurrentLimits.SupplyCurrentLimit = 40; //TODO: find value
    config2.CurrentLimits.SupplyCurrentLimitEnable = true;
    config2.Slot0.kP = 0.0;
    config2.Slot0.kG = 0.0;
    config2.Slot0.kS = 0.0;
    config2.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(config));
    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(config2));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
    motionMagicConfigs.MotionMagicAcceleration = 0.0;
    motionMagicConfigs.MotionMagicJerk = 0.0;
    motionMagicConfigs.MotionMagicExpo_kV = 0.1;
    motionMagicConfigs.MotionMagicExpo_kA = 0.1;
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(motionMagicConfigs));
    PhoenixUtil.tryUntilOk(5, () -> followerMotor.getConfigurator().apply(motionMagicConfigs));
  }
  
  // Updates the inputs of the elevator
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.distance.mut_replace(Inches.of(followerMotor.getPosition().getValue().in(Rotations) * 2 * Math.PI * spoolRadius));
    inputs.rightDist.mut_replace(Inches.of(leaderMotor.getPosition().getValue().in(Rotations) * 2 * Math.PI * spoolRadius));
    inputs.velocity.mut_replace(MetersPerSecond.of(followerMotor.getVelocity().getValue().in(DegreesPerSecond) * 2 * Math.PI * spoolRadius));
    inputs.setpoint.mut_replace(setpoint);
    inputs.supplyCurrent.mut_replace(leaderMotor.getStatorCurrent().getValue());
  }

  // Sets the target distance of the elevator
  @Override
  public void setTarget(Distance target) {
    request = request.withPosition(target.in(Inches)/(2 * Math.PI * spoolRadius));
    leaderMotor.setControl(request);
    followerMotor.setControl(request);
    setpoint = target;
  }

  // Stops the motors of the elevator
  @Override
  public void stop() {
    followerMotor.setControl(new StaticBrake());
    leaderMotor.setControl(new StaticBrake());
  }
}
