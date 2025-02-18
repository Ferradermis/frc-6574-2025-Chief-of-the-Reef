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
    //TalonFXConfiguration leftConfig = new TalonFXConfiguration();
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

    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(config));

    followerMotor.setControl(new Follower(leaderMotor.getDeviceID(), true));

    // Slot0Configs slot0Configs = new Slot0Configs();
    // slot0Configs.kP = 0.0;
    // slot0Configs.kI = 0.0;
    // slot0Configs.kD = 0.0;
    // slot0Configs.kS = 0.0;
    // slot0Configs.kG = 0.0;
    // slot0Configs.kV = 0.0;
    // slot0Configs.kA = 0.0;
    // slot0Configs.GravityType = GravityTypeValue.Elevator_Static;
    // PhoenixUtil.tryUntilOk(5, () -> rightMotor.getConfigurator().apply(slot0Configs));

    MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
    motionMagicConfigs.MotionMagicCruiseVelocity = 0.0;
    motionMagicConfigs.MotionMagicAcceleration = 0.0;
    motionMagicConfigs.MotionMagicJerk = 0.0;
    motionMagicConfigs.MotionMagicExpo_kV = 0.0;
    motionMagicConfigs.MotionMagicExpo_kA = 0.0;
    PhoenixUtil.tryUntilOk(5, () -> leaderMotor.getConfigurator().apply(motionMagicConfigs));
  }
  
  // Updates the inputs of the elevator
  @Override
  public void updateInputs(ElevatorInputs inputs) {
    inputs.distance.mut_replace(Inches.of(followerMotor.getPosition().getValue().in(Rotations) * 2 * Math.PI * spoolRadius));
    inputs.rightDist.mut_replace(Inches.of(leaderMotor.getPosition().getValue().in(Rotations) * 2 * Math.PI * spoolRadius));
    inputs.velocity.mut_replace(MetersPerSecond.of(followerMotor.getVelocity().getValue().in(DegreesPerSecond) * 2 * Math.PI * spoolRadius));
    inputs.setpoint.mut_replace(
        Distance.ofRelativeUnits(
            ((MotionMagicVoltage) leaderMotor.getAppliedControl()).Position, Inches));
    inputs.supplyCurrent.mut_replace(leaderMotor.getStatorCurrent().getValue());
  }

  // Sets the target distance of the elevator
  @Override
  public void setTarget(Distance inches) {
    request = request.withPosition(inches.in(Inches)/(2 * Math.PI * spoolRadius));
    leaderMotor.setControl(request);
  }

  // Stops the motors of the elevator
  @Override
  public void stop() {
    followerMotor.setControl(new StaticBrake());
    leaderMotor.setControl(new StaticBrake());
  }
}
