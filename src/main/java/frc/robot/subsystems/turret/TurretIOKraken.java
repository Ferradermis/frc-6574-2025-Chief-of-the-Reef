package frc.robot.subsystems.turret;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.util.PhoenixUtil;

public class TurretIOKraken implements TurretIO{
    public TalonFX motor;
    public MotionMagicVoltage request;
    public TurretConstants turretConstants;
    private double angleSetpoint = 0;
    
    // Create a new instance of the TurretIOKraken subsystem
    // Creates a new TalonFX motor controller for the turret and a new voltage output request
    public TurretIOKraken (int motorid) {
        motor = new TalonFX(motorid);
        turretConstants = new TurretConstants();
        request = new MotionMagicVoltage(turretConstants.startingAngle);
        configureKrakens();
    }

    // Configures the TalonFX motor controller for the turret
    public void configureKrakens() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Voltage.PeakForwardVoltage = 1.0; //TODO: Probably need to change this value
        config.Voltage.PeakReverseVoltage = -1.0; //TODO: Probably need to change this value
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = 1; 

        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 0.1;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 0.0;
        slot0Configs.kS = 0.0;
        slot0Configs.kG = 0.0;
        slot0Configs.kV = 0.0;
        slot0Configs.kA = 0.0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(slot0Configs));

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 20.0;
        motionMagicConfigs.MotionMagicAcceleration = 20.0;
        motionMagicConfigs.MotionMagicJerk = 0.0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.0;
        motionMagicConfigs.MotionMagicExpo_kA = 0.0;
        
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motionMagicConfigs));
    }

    // Sets the target angle of the turret
    @Override
    public void setTarget(double target) {
        request = request.withPosition(target);
        motor.setControl(request);
        angleSetpoint = target;
    }

    // Updates the inputs of the turret
    @Override
    public void updateInputs(TurretInputs inputs) {
        inputs.angle = motor.getPosition().getValueAsDouble();
        inputs.angularVelocity.mut_replace(motor.getVelocity().getValue());
        inputs.setpoint = angleSetpoint;
        inputs.voltageSetpoint.mut_replace(motor.getMotorVoltage().getValue());
        inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValue());
    }

    // Stops the motor of the turret
    @Override
    public void stop() {
        motor.setControl(new StaticBrake());
    }

    // Sets the voltage of the arm
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    // Gets the constants of the turret
    @Override
    public TurretConstants getConstants() {
        return turretConstants;
    }
}
