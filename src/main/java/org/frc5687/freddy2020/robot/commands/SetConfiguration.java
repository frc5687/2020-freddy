package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.freddy2020.robot.Robot;

public class SetConfiguration extends OutliersCommand {

    private Robot _robot;
    private Robot.Configuration _configuration;

    public SetConfiguration(Robot robot, Robot.Configuration configuration) {
        _robot = robot;
        _configuration = configuration;
        error("Configuration set to " + configuration.name() + " at " + DriverStation.getInstance().getMatchTime());
    }

    @Override
    protected void initialize() {
        _robot.setConfiguration(_configuration);
    }

    @Override
    protected boolean isFinished() {
        return true;
    }

    @Override
    protected void end() {
    }
}


