package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LockingServo extends SubsystemBase {
    public Servo servo;
    
    // Creates a new servo using the provided servo channel
    public LockingServo(int servoChannel) {
        servo = new Servo(servoChannel);
    }

    // Sets the angle of the servo
    public void setAngle(int angle) {
        servo.setAngle(angle);
    }

    // Create a new command to set the angle of the servo
    public Command getNewSetAngleCommand(int a) {
        return new InstantCommand(
            () -> {
            setAngle(a);
            },
            this);
    }

    // called periodically to update the SmartDashboard with the servo angle
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Servo Angle", servo.getAngle());
    }
}
