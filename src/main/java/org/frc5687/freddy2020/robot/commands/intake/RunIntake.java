package org.frc5687.freddy2020.robot.commands.intake;

import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;

public class RunIntake extends OutliersCommand {

    private CargoIntake _intake;
    private OI _oi;

    public RunIntake (Robot robot, CargoIntake intake) {
        _intake = intake;
        _oi = robot.getOI();
        requires(_intake);

    }
    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        double speed = _oi.getRollerSpeed();
        _intake.run(speed);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

}
