package frc.robot.subsystems.pivot;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import edu.wpi.first.units.measure.Angle;

//TODO: Most likely does not work, please verify and fix (I am trying) qwq
public class PivotIONEO implements PivotIO {
    public SparkMax m_pivotMotor;
    public SparkClosedLoopController m_controller;
    public SparkMaxConfig m_pivotConfig;

    public PivotIONEO(int motorId) {
        m_pivotMotor = new SparkMax(motorId, SparkMax.MotorType.kBrushless);
        m_controller = m_pivotMotor.getClosedLoopController();
        m_pivotConfig = new SparkMaxConfig();
    }

    /** Configures the NEO motor */
    public void configureNEO() {
        m_pivotConfig.inverted(false).idleMode(IdleMode.kBrake);
        m_pivotConfig.encoder.positionConversionFactor(1) //TODO: Find correct conversion factor - defaulted at 1 for now :)
            .velocityConversionFactor(1); //TODO: Find correct conversion factor - defaulted at 1 for now :)
        m_pivotConfig.closedLoop.feedbackSensor(FeedbackSensor.kPrimaryEncoder) //TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
            .pid(0.1, 0, 0) //using default slot 0 for this NEO - we will probably not use this slot much or at all
            .outputRange(-1, 1) // same thing here, will probably not use this
            .pid(0, 0, 0, ClosedLoopSlot.kSlot1) //TODO: Find correct PID values - defaulted at 0 for now :)
            .velocityFF(0, ClosedLoopSlot.kSlot1) //TODO: Find correct velocity feedforward value - defaulted at 0 for now :)
            .outputRange(-1, 1, ClosedLoopSlot.kSlot1); 

        m_pivotConfig.softLimit.forwardSoftLimit(0).reverseSoftLimit(0); //TODO: Find correct soft limits - both set to zero for now :)

        m_pivotMotor.configure(m_pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void setTarget(Angle target) {
        m_controller.setReference(target.magnitude(), ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    //TODO: I DO NOT KNOW IF THESE UNIT CONVERSIONS ARE DONE CORRECTLY! PLEASE VERIFY! thank :)
    @Override
    public void updateInputs(PivotInputs inputs) {
        inputs.pivotAngle.mut_replace(m_pivotMotor.getEncoder().getPosition(), Rotations);
        inputs.pivotAngularVelocity.mut_replace(m_pivotMotor.getEncoder().getVelocity(), RotationsPerSecond);
        inputs.pivotSetPoint.mut_replace(Angle.ofRelativeUnits(m_pivotMotor.getAppliedOutput(), Rotations));
        inputs.supplyCurrent.mut_replace(m_pivotMotor.getOutputCurrent(), Amps);
    }
}
