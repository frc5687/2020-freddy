
package org.frc5687.freddy2020.robot.commands.intake;

import com.kauailabs.navx.frc.AHRS;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.HatchIntake;

public class HoldClawOpen extends OutliersCommand {
    private Robot _robot;
    private HatchIntake _hatchIntake;

    private boolean _wasTriggered = false;


    public HoldClawOpen(Robot robot) {
        _robot = robot;
        _hatchIntake = robot.getHatchIntake();
        requires(_hatchIntake);
    }
    @Override
    protected void initialize() {
        _hatchIntake.pointClaw();
        _wasTriggered = _hatchIntake.isHatchDetected() || _hatchIntake.isShockTriggered();
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        _hatchIntake.gripClaw();
    }

    @Override
    protected void execute(){
        if (!_wasTriggered
                && (_hatchIntake.isHatchDetected() || _hatchIntake.isShockTriggered())
                && !_robot.getElevator().isAboveMiddle()) {
            _hatchIntake.gripClaw();
        }
    }
}