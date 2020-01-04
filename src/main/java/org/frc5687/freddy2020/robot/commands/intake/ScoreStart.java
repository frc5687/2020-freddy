package org.frc5687.freddy2020.robot.commands.intake;

import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;
import org.frc5687.freddy2020.robot.subsystems.HatchIntake;

public class ScoreStart extends OutliersCommand {
    private HatchIntake _intake;
    private CargoIntake _cargoIntake;
    private Robot _robot;
    private long _rollerEndTime;
    private long _kickEndTime;

    public ScoreStart(Robot robot) {
        _intake = robot.getHatchIntake();
        _cargoIntake = robot.getCargoIntake();
        _robot = robot;
        requires(_intake);
        requires(_cargoIntake);
    }
    @Override
    protected boolean isFinished() {

        return System.currentTimeMillis() > _rollerEndTime
                && System.currentTimeMillis() > _kickEndTime;
    }
    @Override
    protected void initialize() {
        _rollerEndTime = System.currentTimeMillis() + Constants.Intake.SCORE_ROLLER_MILLIS;
        _kickEndTime = System.currentTimeMillis() + Constants.Intake.SCORE_KICK_MILLIS;
        _intake.raiseWrist();
        _intake.pointClaw();
        _cargoIntake.startRoller();
    }
    @Override
    protected void execute(){
        if (System.currentTimeMillis() > _rollerEndTime) {
            _cargoIntake.stopRoller();
        }
        if (System.currentTimeMillis() > _kickEndTime) {
            _intake.lowerWrist();
            _intake.pointClaw();
        }
    }

    @Override
    protected void end() {
        _intake.lowerWrist();
        _intake.pointClaw();
        _cargoIntake.stopRoller();
    }
}
