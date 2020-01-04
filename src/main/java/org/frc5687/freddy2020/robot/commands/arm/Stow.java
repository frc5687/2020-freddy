package org.frc5687.freddy2020.robot.commands.arm;

import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.Arm;
import static org.frc5687.freddy2020.robot.Constants.Arm.*;

public class Stow extends OutliersCommand {
    private Arm _arm;

    public Stow(Arm arm) {
        _arm = arm;
    }


    @Override
    protected void initialize() {
        super.initialize();
    }

    @Override
    protected void execute() {
        if (!_arm.isRightStowed()) {
            _arm.setRightSpeed(STOW_SPEED);
        }
        if (!_arm.isLeftStowed()) {
            _arm.setLeftSpeed(STOW_SPEED);
        }
    }

    @Override
    protected boolean isFinished() {
        return _arm.isLeftStowed() && _arm.isRightStowed();
    }
}
