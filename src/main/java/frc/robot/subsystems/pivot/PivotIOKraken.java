package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;
import frc.robot.util.PhoenixUtil;

public class PivotIOKraken implements PivotIO{
    public TalonFX motor;
    public CANcoder encoder;
    public MotionMagicVoltage request;
    public PivotConstants pivotConstants;
    private double angleSetpoint = 0;
    
    // Create a new instance of the PivotIOKraken subsystem
    // Creates a new TalonFX motor controller for the pivot and a new voltage output request
    public PivotIOKraken (int motorid) {
        motor = new TalonFX(motorid);
        encoder = new CANcoder(Constants.CANConstants.PIVOT_CANCODER_ID);
        pivotConstants = new PivotConstants();
        request = new MotionMagicVoltage(0.187);
        configureKrakens();
    }

    // Configures the TalonFX motor controller for the pivot
    public void configureKrakens() {
        CANcoderConfiguration cancoderConfig = new CANcoderConfiguration();
        cancoderConfig.MagnetSensor.MagnetOffset = 0.3125; //-0.088;
        cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = 0.5;
 
        PhoenixUtil.tryUntilOk(5, () -> encoder.getConfigurator().apply(cancoderConfig));

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.Voltage.PeakForwardVoltage = 12.0; //TODO: Probably need to change this value
        config.Voltage.PeakReverseVoltage = -12.0; //TODO: Probably need to change this value
        config.CurrentLimits.StatorCurrentLimit = 80.0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 40.0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.Feedback.SensorToMechanismRatio = 1; 
        config.Feedback.RotorToSensorRatio = (45 * 60) / 34;
        config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RemoteCANcoder;
        config.Feedback.FeedbackRemoteSensorID = Constants.CANConstants.PIVOT_CANCODER_ID;
        config.Feedback.FeedbackRotorOffset = 0;
        config.ClosedLoopGeneral.ContinuousWrap = false;

        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));

        Slot0Configs slot0Configs = new Slot0Configs();
        slot0Configs.kP = 120.0;
        slot0Configs.kI = 0.0;
        slot0Configs.kD = 2.0;
        slot0Configs.kS = 0.0;
        slot0Configs.kG = 0.4;
        slot0Configs.kV = 0.0;
        slot0Configs.kA = 0.0;
        slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(slot0Configs));

        MotionMagicConfigs motionMagicConfigs = new MotionMagicConfigs();
        motionMagicConfigs.MotionMagicCruiseVelocity = 50.0;
        motionMagicConfigs.MotionMagicAcceleration = 5.0;
        motionMagicConfigs.MotionMagicJerk = 0.0;
        motionMagicConfigs.MotionMagicExpo_kV = 0.0;
        motionMagicConfigs.MotionMagicExpo_kA = 0.0;
        
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(motionMagicConfigs));
    }

    // Sets the target angle of the pivot
    @Override
    public void setTarget(double target) {
        request = request.withPosition(target);
        motor.setControl(request); //0.187
        angleSetpoint = target;
    }

    // Updates the inputs of the pivot
    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.angle.mut_replace(motor.getPosition().getValue());
        inputs.encoder = encoder.getAbsolutePosition().getValueAsDouble();
        inputs.angularVelocity.mut_replace(motor.getVelocity().getValue());
        inputs.setpoint = angleSetpoint;
        inputs.voltageSetpoint.mut_replace(motor.getMotorVoltage().getValue());
        inputs.error = motor.getClosedLoopError().getValueAsDouble();
        inputs.supplyCurrent.mut_replace(motor.getStatorCurrent().getValue());
    }

    // Stops the motor of the pivot
    @Override
    public void stop() {
        motor.setControl(new StaticBrake());
    }

    // Sets the voltage of the pivot
    @Override
    public void setVoltage(double voltage) {
        motor.setVoltage(voltage);
    }

    // Gets the constants of the pivot
    @Override
    public PivotConstants getConstants() {
        return pivotConstants;
    }
}
