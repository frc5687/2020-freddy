package org.frc5687.freddy2020.robot.commands;

import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;

public class DriveRoller extends OutliersCommand {

    private CargoIntake _intake;
    private OI _oi;

    public DriveRoller(Robot robot) {
        _intake = robot.getCargoIntake();
        _oi = robot.getOI();
        requires(_intake);
    }
    @Override
    protected void execute() {
        double speed = _oi.getRollerSpeed();
        _intake.run(speed);
    }

    @Override
    protected boolean isFinished() {
        return false;
    }
}
