package frc.robot.subsystems.rotate;

import edu.wpi.first.units.measure.Angle;

public class RotateIOSim implements RotateIO{
    private RotateConstants rotateConstants;

    @Override
    public void setTarget(Angle target) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'setTarget'");
    }

    @Override
    public void updateInputs(RotateInputs inputs) {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'updateInputs'");
    }

    @Override
    public void stop() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'stop'");
    }

    @Override
    public RotateConstants getConstants() {
        return rotateConstants;
    }

}
