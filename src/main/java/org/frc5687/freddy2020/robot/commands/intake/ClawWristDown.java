package org.frc5687.freddy2020.robot.commands.intake;

import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;
import org.frc5687.freddy2020.robot.subsystems.HatchIntake;

public class ClawWristDown extends OutliersCommand {
    private HatchIntake _intake;
    private Robot _robot;
    private long _endTime;

    public ClawWristDown(Robot robot) {
        _intake = robot.getHatchIntake();
        _robot = robot;
        requires(_intake);
    }
    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _endTime;
    }
    @Override
    protected void initialize() {
        _endTime = _intake.isDown() ? System.currentTimeMillis() : System.currentTimeMillis() + Constants.Intake.CLAW_LOWER_WRIST_MILLI_SEC;
        _intake.lowerWrist();
        _intake.pointClaw();
    }
    @Override
    protected void execute(){
        _intake.lowerWrist();
        _intake.pointClaw();
    }
}
