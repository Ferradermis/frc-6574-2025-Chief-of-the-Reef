package frc.robot.subsystems.climber;

import static edu.wpi.first.units.Units.*;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.PeriodicFrame;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Servo;

//TODO: Most likely does not work (untested), please verify and fix (I am trying my best to code without a bot qwq)
//TODO: also add servo implementation
public class ClimberIOMotors implements ClimberIO {
    public SparkMax m_climberMotor;
    public SparkClosedLoopController m_controller;
    public SparkMaxConfig m_climberConfig;
    public Servo m_lockingServo;

    //public ClimberIOMotors(int motorId, int servoChannel) {
    public ClimberIOMotors(int motorId) {
        m_climberMotor = new SparkMax(motorId, SparkMax.MotorType.kBrushless);
        m_controller = m_climberMotor.getClosedLoopController();
        m_climberConfig = new SparkMaxConfig();
        //m_lockingServo = new Servo(servoChannel);
    }

    /** Configures the NEO motor */
    public void configureNEO() { //STOP AUTO FORMATTING PLEASE I LIKE MY CODE THE WAY IT IS
        m_climberConfig.inverted(false).idleMode(IdleMode.kBrake);
        m_climberConfig.encoder
            .positionConversionFactor(1) // TODO: Find correct conversion factor - defaulted at 1 for now :)
            .velocityConversionFactor(1); // TODO: Find correct conversion factor - defaulted at 1 for now :)
        m_climberConfig.closedLoop
                .feedbackSensor(FeedbackSensor.kPrimaryEncoder) // TODO: Find correct feedback sensor - defaulted at primary encoder for now :)
                .pid(0.1, 0, 0) // using default slot 0 for this NEO - we will probably not use this slot much or at all
                .outputRange(-1, 1) // same thing here, will probably not use this
                .pid(0.5, 0, 0, ClosedLoopSlot.kSlot1) // TODO: Find correct PID values - defaulted at 0 for now :)
                .velocityFF(0, ClosedLoopSlot.kSlot1) // TODO: Find correct velocity feedforward value - defaulted at 0 for now  :)
                .outputRange(-1, 1, ClosedLoopSlot.kSlot1);

        m_climberConfig
            .softLimit
            .forwardSoftLimit(1)
            .reverseSoftLimit(-1); // TODO: Find correct soft limits - both set to zero for now :)

        m_climberMotor.configure(
            m_climberConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public double getEncoder() {
        return m_climberMotor.getEncoder().getPosition();
    }

    @Override
    public void setClimberTarget(Angle target) {
        m_controller.setReference(target.in(Rotations), ControlType.kPosition, ClosedLoopSlot.kSlot1);
    }

    public void setServoTarget(Angle target) {
        m_lockingServo.set(target.in(Degrees));
    }

    @Override
    public void stop() {
        m_climberMotor.stopMotor();
    }

    // TODO: I DO NOT KNOW IF THESE UNIT CONVERSIONS ARE DONE CORRECTLY! PLEASE VERIFY! thank :)
    @Override
    public void updateInputs(ClimberInputs inputs) {
        inputs.climberAngle.mut_replace(Rotations.of(m_climberMotor.getEncoder().getPosition()));
        inputs.climberAngularVelocity.mut_replace(
            RotationsPerSecond.of(m_climberMotor.getEncoder().getVelocity()));
        inputs.climberSetPoint.mut_replace(
            Angle.ofRelativeUnits(m_climberMotor.getAppliedOutput(), Rotations));
        inputs.supplyCurrent.mut_replace(m_climberMotor.getOutputCurrent(), Amps);
    }
}
