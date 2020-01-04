package org.frc5687.freddy2020.robot.commands.intake;

import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;

public class StopRoller extends OutliersCommand {
    private CargoIntake _intake;

    public StopRoller(CargoIntake intake) {
        _intake = intake;
        requires(_intake);
    }

    @Override
    protected void initialize() {
    }

    @Override
    protected void execute() {
        _intake.stopRoller();
    }



    @Override
    protected boolean isFinished() {
        return true;
    }
}
