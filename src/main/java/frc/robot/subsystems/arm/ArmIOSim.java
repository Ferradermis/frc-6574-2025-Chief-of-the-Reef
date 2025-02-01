package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;

public class ArmIOSim implements ArmIO {
    private ArmFeedforward feedforward;
    private final ProfiledPIDController controller;
    private final SingleJointedArmSim simulator;
    private final ArmConstants armConstants;
    private Voltage appliedVoltage = Volts.mutable(0);

    public ArmIOSim(ArmConstants constants) {
        simulator = new SingleJointedArmSim(
            DCMotor.getNEO(1), 
            constants.gearing, 
            SingleJointedArmSim.estimateMOI(constants.length.in(Meters), constants.weight.in(Kilograms)), 
            constants.length.in(Meters), 
            constants.minimumAngle.in(Radians), 
            constants.maximumAngle.in(Radians), 
            true, 
            constants.startingAngle.in(Radians), 
            0.001,
            0.001);
        feedforward = new ArmFeedforward(
            constants.simGains.kS, 
            constants.simGains.kG, 
            constants.simGains.kV, 
            constants.simGains.kA);
        controller = new ProfiledPIDController(
            constants.simGains.kP, 
            constants.simGains.kI, 
            constants.simGains.kD, 
            new Constraints(constants.maxVelocity.in(DegreesPerSecond), constants.maxAcceleration.in(DegreesPerSecondPerSecond)));
        armConstants = constants;
    }

    @Override
    public void setTarget(Angle target) {
        controller.setGoal(new State(target.in(Degrees), 0));
    }

    private void updateVoltageSetpoint() {
        Angle currentAngle = Radians.of(simulator.getAngleRads());
        Voltage controllerVoltage = Volts.of(controller.calculate(currentAngle.in(Degrees)));
        Voltage feedforwardVoltage = Volts.of(feedforward.calculate(controller.getSetpoint().position, controller.getSetpoint().velocity));
        Voltage effort = controllerVoltage.plus(feedforwardVoltage);
        runVolts(effort);
    }

    private void runVolts(Voltage volts) {
        appliedVoltage = volts;
    }

    @Override
    public void updateInputs(ArmInputs inputs) {
        inputs.angle.mut_replace(
            Degrees.convertFrom(simulator.getAngleRads(), Radians), 
            Degrees);
        inputs.angularVelocity.mut_replace(
            DegreesPerSecond.convertFrom(simulator.getVelocityRadPerSec(), RadiansPerSecond),
            DegreesPerSecond);
        inputs.setpoint.mut_replace(controller.getGoal().position, Degrees);
        inputs.supplyCurrent.mut_replace(simulator.getCurrentDrawAmps(), Amps);
        inputs.torqueCurrent.mut_replace(inputs.supplyCurrent.in(Amps), Amps);
        inputs.voltageSetpoint.mut_replace(appliedVoltage);

        updateVoltageSetpoint();
        simulator.setInputVoltage(appliedVoltage.in(Volts));
        simulator.update(0.02);
    }

    @Override
    public void stop() {
        Angle currentAngle = Radians.of(simulator.getAngleRads());
        controller.reset(currentAngle.in(Degrees));
        runVolts(Volts.of(0));
    }

    @Override
    public ArmConstants getConstants() {
        return armConstants;
    }
}
