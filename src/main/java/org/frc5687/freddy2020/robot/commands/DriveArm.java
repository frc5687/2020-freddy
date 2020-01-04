package org.frc5687.freddy2020.robot.commands;

import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.subsystems.Arm;

public class DriveArm extends OutliersCommand {

    private Arm _arm;
    private OI _oi;

    public DriveArm(Robot robot, Arm arm) {
        _arm = arm;
        _oi = robot.getOI();
        requires(_arm);
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        double speed = _oi.getArmSpeed();

        _arm.setSpeed(speed);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        // Set arm motor speeds to 0 and set break mode?s
    }
}

