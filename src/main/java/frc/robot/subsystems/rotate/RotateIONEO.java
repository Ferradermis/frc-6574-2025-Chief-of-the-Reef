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
import edu.wpi.first.units.measure.Angle;
import frc.robot.subsystems.arm.ArmConstants;
import frc.robot.subsystems.arm.ArmIO.ArmInputs;

public class RotateIONEO implements RotateIO {
    public SparkMax m_motor;
    public SparkClosedLoopController m_motorController;
    public SparkMaxConfig m_motorConfig;
    public RotateConstants rotateConstants;
    
    public RotateIONEO(int leftMotorId) {
        m_motor = new SparkMax(leftMotorId, SparkMax.MotorType.kBrushless);
        m_motorController = m_motor.getClosedLoopController();
        m_motorConfig = new SparkMaxConfig();
    }

    /** Configures the NEO motor */
    public void configureNEO() { 
        //Left motor configs
        m_motorConfig.inverted(false).idleMode(IdleMode.kBrake); //TODO: either the left or right motor will need to be inverted, will find out when I see the robot
        m_motorConfig.encoder
            .positionConversionFactor(1) // TODO: Find correct conversion factor - defaulted at 1 for now :)
            .velocityConversionFactor(1); // TODO: Find correct conversion factor - defaulted at 1 for now :) 
        m_motorConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
                .pid(0.1, 0, 0) // using default slot 0 for this NEO - we will probably not use this slot much or at all
                .outputRange(-1, 1) // same thing here, will probably not use this
                .pid(0, 0, 0, ClosedLoopSlot.kSlot1) // TODO: Find correct PID values - defaulted at 0 for now :)
                .velocityFF(0, ClosedLoopSlot.kSlot1) // TODO: Find correct velocity feedforward value - defaulted at 0 for now  :)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        m_motorConfig
            .softLimit
            .forwardSoftLimit(0)
            .reverseSoftLimit(0); // TODO: Find correct soft limits - both set to zero for now :)

        m_motor.configure(
            m_motorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setTarget(Angle target) {
        m_motorController.setReference(target.magnitude(), ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    @Override
    public void updateInputs(RotateInputs inputs) {
        inputs.angle.mut_replace(
            Degrees.convertFrom(m_motor.getEncoder().getPosition(), Radians), 
            Degrees);
        inputs.angularVelocity.mut_replace(
            DegreesPerSecond.convertFrom(m_motor.getEncoder().getVelocity(), RadiansPerSecond),
            DegreesPerSecond);
        inputs.setpoint.mut_replace(m_motor.getAppliedOutput(), Degrees);
        inputs.supplyCurrent.mut_replace(m_motor.getOutputCurrent(), Amps);
        inputs.torqueCurrent.mut_replace(inputs.supplyCurrent.in(Amps), Amps);
        inputs.voltageSetpoint.mut_replace(m_motor.getBusVoltage(), Volts);
    }

    @Override
    public void stop() {
        m_motor.stopMotor();
    }

    @Override
    public RotateConstants getConstants() {
        return rotateConstants;
    }
}
