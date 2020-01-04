package org.frc5687.freddy2020.robot.commands.intake;

import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;

import static org.frc5687.freddy2020.robot.Constants.Intake.ROLLER_SPEED;
import static org.frc5687.freddy2020.robot.subsystems.CargoIntake.RollerMode.DONE;
import static org.frc5687.freddy2020.robot.subsystems.CargoIntake.RollerMode.WAITING;

public class StartRoller extends OutliersCommand {
    private CargoIntake _intake;
    private boolean _stopWhenSecured;
    private long _stopTime;

    public StartRoller(CargoIntake intake, boolean stopWhenSecured) {
        _intake = intake;
        _stopWhenSecured = stopWhenSecured;
        requires(_intake);
    }
    
    @Override
    protected void initialize() {
        _intake.setRollerMode(CargoIntake.RollerMode.RUNNING);
        _intake.startRoller();
    }

    @Override
    protected void execute() {
        CargoIntake.RollerMode rollerMode = _intake.getRollerMode();
        switch(rollerMode) {
            case RUNNING:
                _intake.run(ROLLER_SPEED);
                if (_intake.isBallDetected()) {
                    _intake.run(0);
                    _stopTime = System.currentTimeMillis() + Constants.Intake.ROLLER_TIME_MILLI_SEC;
                    _intake.setRollerMode(WAITING);
                }
                break;
            case WAITING:
                _intake.run(ROLLER_SPEED);
                if (System.currentTimeMillis() > _stopTime) {
                    _intake.setRollerMode(DONE);
                }
        }
    }

    @Override
    protected void end() {
        if (_intake.getRollerMode() == CargoIntake.RollerMode.DONE) {
            _intake.stopRoller();
        }
    }


    @Override
    protected boolean isFinished() {
        if (!_stopWhenSecured) {
            return true;
        }
        return _intake.isBallDetected();
    }
}
