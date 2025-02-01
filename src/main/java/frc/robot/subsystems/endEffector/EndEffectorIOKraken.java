package frc.robot.subsystems.endEffector;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.util.PhoenixUtil;

public class EndEffectorIOKraken implements EndEffectorIO {
    public VoltageOut request;
    public TalonFX motor;

    public EndEffectorIOKraken(int motorId) {
        motor = new TalonFX(motorId);
        request = new VoltageOut(0.0);

        motor.setControl(request);
        configureTalons();
    }

    private void configureTalons() {
        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        config.CurrentLimits.StatorCurrentLimit = 0;
        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.SupplyCurrentLimit = 0;
        config.CurrentLimits.SupplyCurrentLimitEnable = true;
        PhoenixUtil.tryUntilOk(5, () -> motor.getConfigurator().apply(config));
    }

    @Override
    public void updateInputs(EndEffectorInputs inputs) {
        inputs.angularVelocity.mut_replace(motor.getVelocity().getValue());
        inputs.voltageSetpoint.mut_replace(
        Voltage.ofRelativeUnits(
            ((VoltageOut) motor.getAppliedControl()).Output, Volts));
        inputs.voltage.mut_replace(motor.getMotorVoltage().getValue());
        inputs.supplyCurrent.mut_replace(motor.getStatorCurrent().getValue());
    }

    @Override
    public void setTarget(Voltage target) {
        request = request.withOutput(target);
        motor.setControl(request);
    }

    @Override
    public void stop() {
        motor.setControl(new StaticBrake());
    }
}
