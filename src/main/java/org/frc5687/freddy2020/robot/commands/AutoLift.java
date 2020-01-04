package org.frc5687.freddy2020.robot.commands;

import org.frc5687.freddy2020.robot.subsystems.Arm;
import org.frc5687.freddy2020.robot.subsystems.Stilt;

public class AutoLift extends OutliersCommand {
    private Arm _arm;
    private Stilt _stilt;

    private double armSpeed = 0.9;
    private double stiltSpeed = 0.6;

    public AutoLift(Arm arm, Stilt stilt) {
        _arm = arm;
        _stilt = stilt;
        requires(_arm);
        requires(_stilt);
    }


    @Override
    protected void execute() {
        _arm.setSpeed(armSpeed);
        _stilt.setLifterSpeed(stiltSpeed);
    }

    @Override
    protected void end() {
        _arm.setSpeed(0.0);
        _stilt.setLifterSpeed(0.0);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
