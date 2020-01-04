package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.intake.PointClaw;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;

import static org.frc5687.freddy2020.robot.Constants.Intake.*;

public class EjectCargo extends OutliersCommand {
    private CargoIntake _cargoIntake;
    private long _stopMillis;

    public EjectCargo(CargoIntake cargoIntake) {
        _cargoIntake = cargoIntake;
    }

    @Override
    protected void initialize() {
        _stopMillis = System.currentTimeMillis() + CARGO_EJECT_MILLIS;
    }

    @Override
    protected void execute() {
        _cargoIntake.setRollerSpeed(CARGO_EJECT_SPEED);
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() >= _stopMillis;
    }

    @Override
    protected void end() {
        _cargoIntake.setRollerSpeed(0);
    }
}
