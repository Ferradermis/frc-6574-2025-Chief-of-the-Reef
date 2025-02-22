package frc.robot.subsystems.rotate;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import static edu.wpi.first.units.Units.*;

public class RotateIONEO implements RotateIO {
    public SparkMax m_motor;
    public SparkClosedLoopController m_motorController;
    public SparkMaxConfig m_motorConfig;
    public RotateConstants rotateConstants;
    private double setpoint = 0.0;
    
    // Create a new instance of the RotateIONEO subsystem
    // Creates a new spark max using the provided motor id and creates a new motor controller and config
    public RotateIONEO(int motorId) {
        m_motor = new SparkMax(motorId, SparkMax.MotorType.kBrushless);
        m_motorController = m_motor.getClosedLoopController();
        m_motorConfig = new SparkMaxConfig();
        rotateConstants = new RotateConstants();
        configureNEO();
    }

    /** Configures the NEO motor */
    public void configureNEO() { 
        //Left motor configs
        m_motorConfig.inverted(false).idleMode(IdleMode.kBrake); //TODO: either the left or right motor will need to be inverted, will find out when I see the robot
        m_motorConfig.encoder
            // .positionConversionFactor(360/12.94) // TODO: Find correct conversion factor - defaulted at 1 for now :)
            // .velocityConversionFactor(360/12.94/60); // TODO: Find correct conversion factor - defaulted at 1 for now :) 
            .positionConversionFactor(1)
            .velocityConversionFactor(1);
        m_motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kAbsoluteEncoder) // TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
               // .pid(0.1, 0, 0) // using default slot 0 for this NEO - we will probably not use this slot much or at all
                .outputRange(-1, 1) // same thing here, will probably not use this
                .pid(2.01, 0, 0, ClosedLoopSlot.kSlot0) // TODO: Find correct PID values - defaulted at 0 for now :)
                .velocityFF(0, ClosedLoopSlot.kSlot0) // TODO: Find correct velocity feedforward value - defaulted at 0 for now  :)
                .outputRange(-2, 2, ClosedLoopSlot.kSlot0);
        m_motorConfig.smartCurrentLimit(40);
        m_motorConfig
            .softLimit
            .forwardSoftLimit(10)
            .reverseSoftLimit(-10); // TODO: Find correct soft limits - both set to zero for now :)

        m_motor.configure(
            m_motorConfig, ResetMode.kNoResetSafeParameters, PersistMode.kPersistParameters);

            
    }

    // Sets the target angle of the rotate subsystem
    @Override
    public void setTarget(double target) {
        m_motorController.setReference(target, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        System.out.println(m_motor.getAbsoluteEncoder().getPosition());
        setpoint = target;
    }

    // Sets the voltage of the rotate
    @Override
    public void setVoltage(double voltage) {
        m_motor.setVoltage(voltage);
    }

    // Updates the inputs of the rotate subsystem
    @Override
    public void updateInputs(RotateInputs inputs) {
        inputs.angle = m_motor.getAbsoluteEncoder().getPosition();
            //Degrees.convertFrom(m_motor.getAbsoluteEncoder().getPosition(), Rotations), Degrees);
        //m_motor.getAbsoluteEncoder().getPosition(), Rotations);
        inputs.angularVelocity.mut_replace(
            DegreesPerSecond.of(m_motor.getAbsoluteEncoder().getVelocity()));
        inputs.setpoint = setpoint; //.mut_replace(Degrees.of(setpoint));
        inputs.supplyCurrent.mut_replace(m_motor.getOutputCurrent(), Amps);
        inputs.torqueCurrent.mut_replace(m_motor.getAppliedOutput(), Amps);
        inputs.voltageSetpoint.mut_replace(m_motor.getBusVoltage(), Volts); // TODO: Incorrect value getting retrieved here, correct this eventually
    }

    // Stops the motor of the rotate subsystem
    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    // Returns the rotate constants
    @Override
    public RotateConstants getConstants() {
        return rotateConstants;
    }
}
