package frc.robot.subsystems;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LockingServo extends SubsystemBase {
    private LockingServo servo;
    
    public LockingServo(int servoChannel) {
        servo = new LockingServo(servoChannel);
    }

    public void setAngle(int angle) {
        servo.setAngle(angle);
    }

    public Command getNewSetAngleCommand(int a) {
        return new InstantCommand(
            () -> {
            setAngle(a);
            },
            this);
    }

    @Override
    public void periodic() {
        
    }
}
