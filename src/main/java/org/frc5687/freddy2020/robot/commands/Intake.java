package org.frc5687.freddy2020.robot.commands;

import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.intake.GripClaw;

public class Intake extends OutliersCommand{
    private Robot _robot;

    public Intake(Robot robot) {
        _robot = robot;
    }

    @Override
    protected void initialize() {
        if (_robot.getConfiguration() == Robot.Configuration.hatch) {
            (new IntakeHatch(_robot)).start();
        } else if (_robot.getConfiguration() == Robot.Configuration.cargo) {
            (new IntakeCargo(_robot)).start();
        }
    }

    @Override
    public void execute() {
    }

    @Override
    protected boolean isFinished() {
        return true;
    }
}

