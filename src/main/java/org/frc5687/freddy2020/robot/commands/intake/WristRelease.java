package org.frc5687.freddy2020.robot.commands.intake;

import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;

public class WristRelease extends OutliersCommand {
    public CargoIntake _intake;

    public WristRelease(CargoIntake intake) {
        _intake = intake;
        requires(_intake);
    }
    @Override
    protected boolean isFinished() {
        return true;
    }
    @Override
    protected void initialize() {
    }
    @Override
    protected void execute(){
        _intake.releaseWrist();
    }
}
