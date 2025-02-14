package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotState;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {
  private ElevatorIO elevatorIO;
  ElevatorInputsAutoLogged loggedElevator = new ElevatorInputsAutoLogged();

  // Create a new instance of the Elevator subsystem
  // Grabs the IO layer for the Elevator subsystem, could be a simulation or real hardware
  public Elevator(ElevatorIO io) {
    elevatorIO = io;
    loggedElevator.distance = Meters.mutable(0);
    loggedElevator.velocity = MetersPerSecond.mutable(0);
    loggedElevator.setpoint = Meters.mutable(0);
    loggedElevator.supplyCurrent = Amps.mutable(0);
    loggedElevator.torqueCurrent = Amps.mutable(0);
    loggedElevator.voltageSetpoint = Volts.mutable(0);

    // Set the elevator source in the visualizer
    RobotState.getInstance().setElevatorSource(loggedElevator.distance);
  }

  // Set the distance of the elevator
  public void setDistance(Distance target) {
    elevatorIO.setTarget(target);
  }

  // Create a new command to set the distance of the elevator
  public Command getNewSetDistanceCommand(int distance) {
    return new InstantCommand(
        () -> {
          setDistance(Meters.of(distance));
        },
        this);
  }

  // Called periodically to update the Elevator subsystem with the new inputs and log them
  @Override
  public void periodic() {
    elevatorIO.updateInputs(loggedElevator);
    Logger.processInputs("RobotState/Elevator", loggedElevator);
  }
}
