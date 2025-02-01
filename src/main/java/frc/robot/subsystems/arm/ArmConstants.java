package frc.robot.subsystems.arm;

import java.util.function.Consumer;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularAcceleration;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Mass;
import edu.wpi.first.units.measure.MutAngle;
import frc.robot.util.CanDef;
import frc.robot.util.Gains;

public abstract class ArmConstants {
    public CanDef leaderProfile;

    public Gains simGains;

    public Gains NEOGains;

    public AngularVelocity maxVelocity;
    public AngularAcceleration maxAcceleration;
    public double maxJerk;
    public Current torqueCurrentLimit;
    public Current supplyCurrentLimit;
    public Current forwardTorqueLimit;
    public Current reverseTorqueLimit;

    public int numMotors;
    public double gearing;
    public Distance length;
    public Mass weight;
    public DCMotor motors;
    public Angle maximumAngle;
    public Angle minimumAngle;
    public Angle startingAngle;

    public Distance xPosition;
    public Distance yPosition;
    public Distance zPosition;
    public Angle pitchModifier;

    public String loggedName;

    public Consumer<MutAngle> mechanismSimCallback;
}
