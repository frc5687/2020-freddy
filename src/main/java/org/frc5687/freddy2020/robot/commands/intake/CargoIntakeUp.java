package org.frc5687.freddy2020.robot.commands.intake;

import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;

public class CargoIntakeUp extends OutliersCommand {
    private CargoIntake _cargoIntake;
    private Robot _robot;
    private long _stopTime;

    public CargoIntakeUp(CargoIntake cargoIntake) {
        _cargoIntake = cargoIntake;
        requires(_cargoIntake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _stopTime;
    }
    @Override
    protected void initialize() {
        _stopTime = _cargoIntake.isUp() ?  System.currentTimeMillis() : System.currentTimeMillis() + Constants.Intake.RAISE_WRIST_MILLI_SEC;
        _cargoIntake.raiseWrist();
    }
    @Override
    protected void execute(){
        _cargoIntake.raiseWrist();
    }
}
