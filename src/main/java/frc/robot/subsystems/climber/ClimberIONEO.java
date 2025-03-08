package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;

public class ClimberIONEO implements ClimberIO {
    public SparkMax m_climberMotor;
    public SparkClosedLoopController m_controller;
    public SparkMaxConfig m_climberConfig;
    private double setpoint = 0;

    // Create a new instance of the RotateIONEO subsystem
    // Creates a new spark max using the provided motor id and creates a new motor controller and config
    public ClimberIONEO(int motorId) {
        m_climberMotor = new SparkMax(motorId, SparkMax.MotorType.kBrushless);
        m_controller = m_climberMotor.getClosedLoopController();
        m_climberConfig = new SparkMaxConfig();
    }

    /** Configures the NEO motor */
    public void configureNEO() {
        m_climberMotor.configure(
            m_climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_climberConfig.inverted(false).idleMode(IdleMode.kBrake);
        m_climberConfig.encoder
            .positionConversionFactor(360/675) // TODO: Find correct conversion factor
            .velocityConversionFactor(360/675/60); // TODO: Find correct conversion factor
        m_climberConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
                .pid(0.1, 0, 0) // using default slot 0 for this NEO - we will probably not use this slot much or at all
                .outputRange(-1, 1) // same thing here, will probably not use this
                .pid(0.5, 0, 0, ClosedLoopSlot.kSlot1) // TODO: Find correct PID values
                .velocityFF(0, ClosedLoopSlot.kSlot1) // TODO: Find correct velocity feedforward value - defaulted at 0 for now  :)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        m_climberConfig.smartCurrentLimit(40);

        m_climberConfig
            .softLimit
            .forwardSoftLimit(20000)
            .reverseSoftLimit(-20000); // TODO: Find correct soft limits - both set to zero for now :)
    }

    // Gets the encoder position of the NEO motor
    public double getEncoder() {
        return m_climberMotor.getEncoder().getPosition();
    }

    // Sets the target angle of the climber
    @Override
    public void setClimberTarget(double target) {
        m_controller.setReference(target, ControlType.kPosition, ClosedLoopSlot.kSlot1);
        setpoint = target;
    }

    // Stops the motor of the climber
    @Override
    public void stop() {
        m_climberMotor.stopMotor();
    }

    // Sets the voltage of the climber
    @Override
    public void setVoltage(double voltage) {
        m_climberMotor.setVoltage(voltage);
        //m_lockingServo.set(sAngle);
    }

    // Updates the inputs of the climber
    @Override
    public void updateInputs(ClimberInputs inputs) {
        //inputs.climberAngle.mut_replace(Degrees.of(m_climberMotor.getEncoder().getPosition()));
        inputs.climberAngle = m_climberMotor.getEncoder().getPosition();
        inputs.climberAngularVelocity.mut_replace(
            DegreesPerSecond.of(m_climberMotor.getEncoder().getVelocity()));
        // inputs.climberSetPoint.mut_replace(
        //     Degrees.of(m_climberMotor.getAppliedOutput()));
        inputs.climberSetpoint = setpoint;
        inputs.supplyCurrent.mut_replace(m_climberMotor.getOutputCurrent(), Amps);
    }
}
