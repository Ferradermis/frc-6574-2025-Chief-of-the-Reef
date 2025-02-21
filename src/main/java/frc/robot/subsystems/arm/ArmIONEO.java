package frc.robot.subsystems.arm;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;

public class ArmIONEO implements ArmIO {
    public SparkMax m_motor;
    public SparkClosedLoopController m_motorController;
    public SparkMaxConfig m_motorConfig;
    public ArmConstants armConstants;

    private double setpoint = 0.0;
    
    //TODO: Delete and rewrite this class to use a kraken instead (YIPPEE)
    // Create a new instance of the ArmIONEO subsystem
    // Creates a new spark max using the provided motor id and creates a new motor controller and config
    public ArmIONEO(int leftMotorId) {
        m_motor = new SparkMax(leftMotorId, SparkMax.MotorType.kBrushless);
        m_motorController = m_motor.getClosedLoopController();
        m_motorConfig = new SparkMaxConfig();
        armConstants = new ArmConstants();
        configureNEO();
    }

    /** Configures the NEO motor */
    public void configureNEO() { 
        //Left motor configs
        m_motorConfig.inverted(false).idleMode(IdleMode.kCoast);
        m_motorConfig.encoder
            .positionConversionFactor(1) // TODO: Find correct conversion factor - defaulted at 1 for now :)
            .velocityConversionFactor(1); // TODO: Find correct conversion factor - defaulted at 1 for now :) 
        m_motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
                .pid(0.5, 0, 0) // using default slot 0 for this NEO - we will probably not use this slot much or at all
                .outputRange(-1, 1) // same thing here, will probably not use this
                .pid(0.5, 0, 0, ClosedLoopSlot.kSlot1) // TODO: Find correct PID values - defaulted at 0 for now :)
                .velocityFF(0, ClosedLoopSlot.kSlot1) // TODO: Find correct velocity feedforward value - defaulted at 0 for now  :)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        m_motorConfig.softLimit
            .forwardSoftLimit(1)
            .reverseSoftLimit(-1); // TODO: Find correct soft limits - both set to zero for now :)

        m_motor.configure(
            m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    // Sets the target angle of the arm
    @Override
    public void setTarget(double target) {
        m_motorController.setReference(target, ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    // Sets the voltage of the arm
    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    // Updates the inputs of the arm
    @Override
    public void updateInputs(ArmInputs inputs) {
        //inputs.angle.mut_replace(m_motor.getEncoder().getPosition(), Rotations);
        inputs.angle = m_motor.getAbsoluteEncoder().getPosition();
        inputs.angularVelocity.mut_replace(
            DegreesPerSecond.convertFrom(m_motor.getEncoder().getVelocity(), RadiansPerSecond),
            DegreesPerSecond);
        //inputs.setpoint.mut_replace(m_motor.getAppliedOutput(), Degrees);
        inputs.setpoint = setpoint;
        inputs.supplyCurrent.mut_replace(m_motor.getOutputCurrent(), Amps);
        inputs.torqueCurrent.mut_replace(m_motor.getAppliedOutput(), Amps);
        inputs.voltageSetpoint.mut_replace(m_motor.getBusVoltage(), Volts);
    }

    // Stops the motor of the arm
    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    // Gets the constants of the arm
    @Override
    public ArmConstants getConstants() {
        return armConstants;
    }
    
}
