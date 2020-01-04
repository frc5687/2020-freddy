package org.frc5687.freddy2020.robot.commands.intake;

import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.CargoMode;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;
import org.frc5687.freddy2020.robot.subsystems.HatchIntake;

public class Eject extends OutliersCommand {
    public HatchIntake _hatchIntake;
    public CargoIntake _cargoIntake;
    public Robot _robot;

    private long _endTime;
    private boolean _ejectCargo;
    private boolean _ejectHatch;

    public Eject(Robot robot) {
        _robot = robot;
        _hatchIntake = robot.getHatchIntake();
        _cargoIntake = robot.getCargoIntake();
        requires(_hatchIntake);
    }

    public Eject(Robot robot, CargoIntake cargoIntake) {
        this(robot);
    }

    @Override
    protected void initialize() {
        super.initialize();
        if (_robot.getConfiguration()== Robot.Configuration.cargo) {
            _ejectCargo = true;
            _ejectHatch = false;
        } else if (_robot.getConfiguration()== Robot.Configuration.hatch) {
            _ejectCargo = false;
            _ejectHatch = true;
        } else {
            _ejectCargo = true;
            _ejectHatch = true;
        }

        _endTime = System.currentTimeMillis();
        if (_ejectCargo) {
            _cargoIntake.setRollerSpeed(Constants.Intake.CARGO_EJECT_SPEED);
            _endTime = System.currentTimeMillis() + Constants.Intake.CARGO_EJECT_MILLIS;
        }
        if (_ejectHatch) {
            _hatchIntake.pointClaw();
        }
    }


    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _endTime;
    }

    @Override
    protected void execute(){
        if (_ejectCargo) {
            _cargoIntake.setRollerSpeed(Constants.Intake.CARGO_EJECT_SPEED);
        }
    }

    @Override
    protected void end() {
        super.end();
        _cargoIntake.setRollerSpeed(0);
        _cargoIntake.stopRoller();
    }
}
