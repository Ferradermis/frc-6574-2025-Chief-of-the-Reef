package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;

import edu.wpi.first.wpilibj.DutyCycleEncoder;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class ArmIOKraken implements ArmIO{
    public TalonFX motor;
    public CANcoder encoder;
    public PositionVoltage request;
    public ArmConstants armConstants;
    private double angleSetpoint = 0;
    
    // Create a new instance of the ArmIOKraken subsystem
    // Creates a new TalonFX motor controller for the arm and a new voltage output request
    public ArmIOKraken (int motorid) {
        motor = new TalonFX(motorid);
        encoder = new CANcoder(Constants.CANConstants.ARM_CANCODER_ID);
        armConstants = new ArmConstants();
        request = new PositionVoltage(armConstants.startingAngle);
        configureKrakens();
    }

    // Configures the TalonFX motor controller for the arm
    public void configureKrakens() {
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = 0; //-0.842;
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;

        PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(cancoderConfig));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        config.Voltage.PeakForwardVoltage = 2.0; //TODO: Probably need to change this value
        config.Voltage.PeakReverseVoltage = -2.0; //TODO: Probably need to change this value
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = ((9 * 60 * 48)/(34 * 93)) * 0.787;
        config.Feedback.RotorToSensorRatio = 1;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = Constants.CANConstants.ARM_CANCODER_ID;
        config.Feedback.FeedbackRotorOffset = 0;
        config.ClosedLoopGeneral.ContinuousWrap = false;

        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 8;
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

    // Sets the target angle of the arm
    @Override
    public void setTarget(double target) {
        request = request.withPosition(target);
        motor.setControl(request);
        angleSetpoint = target;
    }

    // Updates the inputs of the arm
    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.angle.mut_replace(motor.getPosition().getValue());
        inputs.encoder = encoder.getAbsolutePosition().getValueAsDouble();
        inputs.angularVelocity.mut_replace(motor.getVelocity().getValue());
        inputs.setpoint = angleSetpoint;
        inputs.voltageSetpoint.mut_replace(motor.getMotorVoltage().getValue());
        inputs.error = motor.getClosedLoopError().getValueAsDouble();
        inputs.supplyCurrent.mut_replace(motor.getSupplyCurrent().getValue());
    }

    // Stops the motor of the arm
    @Override
    public void stop() {
        motor.setControl(new StaticBrake());
    }

    // Sets the voltage of the arm
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    // Gets the constants of the arm
    @Override
    public ArmConstants getConstants() {
        return armConstants;
    }
}
