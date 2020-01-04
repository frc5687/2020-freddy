package org.frc5687.freddy2020.robot.commands.stilt;

import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.Stilt;

public class DriveStilt extends OutliersCommand {
    private Stilt _stilt;
    private OI _oi;

    public DriveStilt(Stilt stilt, OI oi) {
        _stilt = stilt;
        _oi = oi;
        requires(_stilt);
    }

    @Override
    protected void execute() {
        // Get value from OI
        double lifterSpeed = _oi.getStiltSpeed();
        double whelieSpeed = _oi.getWheelieSpeed();

        // write to stilt
        _stilt.setLifterSpeed(lifterSpeed);
        _stilt.setWheelieSpeed(whelieSpeed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
