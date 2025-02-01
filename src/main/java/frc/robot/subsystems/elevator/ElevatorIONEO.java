package frc.robot.subsystems.elevator;

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
import edu.wpi.first.units.measure.Distance;

public class ElevatorIONEO implements ElevatorIO {
    public SparkMax m_leftMotor;
    public SparkClosedLoopController m_leftMotorController;
    public SparkMaxConfig m_leftMotorConfig;
    public SparkMax m_rightMotor;
    public SparkClosedLoopController m_rightMotorController;
    public SparkMaxConfig m_rightMotorConfig;

    public ElevatorIONEO(int leftMotorId, int rightMotorId) {
        m_leftMotor = new SparkMax(leftMotorId, SparkMax.MotorType.kBrushless);
        m_leftMotorController = m_leftMotor.getClosedLoopController();
        m_leftMotorConfig = new SparkMaxConfig();
        m_rightMotor = new SparkMax(rightMotorId, SparkMax.MotorType.kBrushless);
        m_rightMotorController = m_rightMotor.getClosedLoopController();
        m_rightMotorConfig = new SparkMaxConfig();
    }

    /** Configures the NEO motor */
    public void configureNEO() { 
        //Left motor configs
        m_leftMotorConfig.inverted(false).idleMode(IdleMode.kBrake); //TODO: either the left or right motor will need to be inverted, will find out when I see the robot
        m_leftMotorConfig.encoder
            .positionConversionFactor(1) // TODO: Find correct conversion factor - defaulted at 1 for now :)
            .velocityConversionFactor(1); // TODO: Find correct conversion factor - defaulted at 1 for now :) 
        m_leftMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
                .pid(0.1, 0, 0) // using default slot 0 for this NEO - we will probably not use this slot much or at all
                .outputRange(-1, 1) // same thing here, will probably not use this
                .pid(0, 0, 0, ClosedLoopSlot.kSlot1) // TODO: Find correct PID values - defaulted at 0 for now :)
                .velocityFF(0, ClosedLoopSlot.kSlot1) // TODO: Find correct velocity feedforward value - defaulted at 0 for now  :)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        m_leftMotorConfig
            .softLimit
            .forwardSoftLimit(0)
            .reverseSoftLimit(0); // TODO: Find correct soft limits - both set to zero for now :)

        m_leftMotor.configure(
            m_leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //Right motor configs
        m_rightMotorConfig.inverted(false).idleMode(IdleMode.kBrake);
        m_rightMotorConfig.encoder
            .positionConversionFactor(1) // TODO: Find correct conversion factor - defaulted at 1 for now :)
            .velocityConversionFactor(1); // TODO: Find correct conversion factor - defaulted at 1 for now :)
        m_rightMotorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
                .pid(0.1, 0, 0) // using default slot 0 for this NEO - we will probably not use this slot much or at all
                .outputRange(-1, 1) // same thing here, will probably not use this
                .pid(0, 0, 0, ClosedLoopSlot.kSlot1) // TODO: Find correct PID values - defaulted at 0 for now :)
                .velocityFF(0, ClosedLoopSlot.kSlot1) // TODO: Find correct velocity feedforward value - defaulted at 0 for now  :)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        m_rightMotorConfig
            .softLimit
            .forwardSoftLimit(0)
            .reverseSoftLimit(0); // TODO: Find correct soft limits - both set to zero for now :)

        m_rightMotor.configure(
            m_leftMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setTarget(Distance target) {
        m_leftMotorController.setReference(target.magnitude(), ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    // TODO: I DO NOT KNOW IF THESE UNIT CONVERSIONS ARE DONE CORRECTLY! PLEASE VERIFY! thank :)
    @Override
    public void updateInputs(ElevatorInputs inputs) {
        inputs.distance.mut_replace(m_leftMotor.getEncoder().getPosition(), Meters);
        inputs.velocity.mut_replace(
            m_leftMotor.getEncoder().getVelocity(), MetersPerSecond);
        inputs.setpoint.mut_replace(m_leftMotor.getAppliedOutput(), Meters);
        inputs.supplyCurrent.mut_replace(m_leftMotor.getOutputCurrent(), Amps);
    }

    @Override
    public void stop() {
        m_leftMotor.stopMotor();
        m_rightMotor.stopMotor();
    }
    
}
