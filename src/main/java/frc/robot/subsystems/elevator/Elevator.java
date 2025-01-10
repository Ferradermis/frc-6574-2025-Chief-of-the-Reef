package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.MutDistance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Elevator extends SubsystemBase{
    private final ElevatorIO io;
    private MutDistance setpoint = Inches.mutable(0);

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        this.io.runSetpoint(setpoint);
    }

    public Command setSetpoint(MutDistance setpoint) {
        return runOnce(() -> this.setpoint = setpoint);
    }
}
