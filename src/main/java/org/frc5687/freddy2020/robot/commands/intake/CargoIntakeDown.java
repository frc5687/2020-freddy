package org.frc5687.freddy2020.robot.commands.intake;

import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;

public class CargoIntakeDown extends OutliersCommand {
    private CargoIntake _cargoIntake;
    private long _endTime;

    public CargoIntakeDown(CargoIntake cargoIntake) {
        _cargoIntake = cargoIntake;
        requires(_cargoIntake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _endTime;
    }
    @Override
    protected void initialize() {
        _endTime = _cargoIntake.isDown() ? System.currentTimeMillis() : System.currentTimeMillis() + Constants.Intake.LOWER_WRIST_MILLI_SEC;
        _cargoIntake.lowerWrist();
    }
    @Override
    protected void execute(){
        _cargoIntake.lowerWrist();
    }
}
