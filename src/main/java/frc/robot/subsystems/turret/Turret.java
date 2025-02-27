package frc.robot.subsystems.turret;

import org.littletonrobotics.junction.Logger;
import static edu.wpi.first.units.Units.*;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Turret extends SubsystemBase{
    private TurretIO turretIO;
    private TurretConstants turretConstants;
    TurretInputsAutoLogged loggedTurret = new TurretInputsAutoLogged();

    // Create a new instance of the Turret subsystem
    // Grabs the IO layer for the Turret subsystem, could be a simulation or real hardware
    public Turret(TurretIO io) {
        turretIO = io;
        loggedTurret.angle = 0; //Degrees.mutable(0);
        loggedTurret.angularVelocity = DegreesPerSecond.mutable(0);
        loggedTurret.setpoint = 0; //Degrees.mutable(0);
        loggedTurret.supplyCurrent = Amps.mutable(0);
        loggedTurret.torqueCurrent = Amps.mutable(0);
        loggedTurret.voltageSetpoint = Volts.mutable(0);

        turretConstants = turretIO.getConstants();
        //turretConstants.mechanismSimCallback.accept(loggedTurret.angle);
    }

    // Set the angle of the Turret subsystem
    public void setAngle(double angle) {
        turretIO.setTarget(angle);
        System.out.println("Setting Turret(setAngle):" + angle);
    }

    // Create a new command to set the angle of the Turret subsystem
    public Command getNewSetAngleCommand(double i) {
        return new InstantCommand(
            () -> {
                setAngle(i);
                System.out.println("Setting Turret(getNewSetAngleCommand):" + i);
            },
            this);
    }

    public Command stopMotors() {
        return new InstantCommand(
            () -> {
                turretIO.stop();
            },
            this);
    }

    public Command reset() {
        return new InstantCommand(
            () -> {
                turretIO.resetEncoder();
            },
            this);
    }

    // Create a new command to set the voltage of the Turret subsystem
    public Command setVoltageTest(double voltage) {
        return new InstantCommand(
            () -> {
              turretIO.setVoltage(voltage);
            },
            this);
      }

    // Called periodically to update the Turret subsystem with the new inputs and log them
    @Override
    public void periodic() {
        turretIO.updateInputs(loggedTurret);
        Logger.processInputs("RobotState/" + turretConstants.loggedName, loggedTurret);
    }
}
