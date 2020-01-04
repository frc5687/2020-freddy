package org.frc5687.freddy2020.robot.commands;

import org.frc5687.freddy2020.robot.subsystems.Lights;

public class DriveLights extends OutliersCommand {
    private Lights _lights;
    private long _cycle = 0;

    public DriveLights(Lights lights) {
        _lights = lights;
        requires(_lights);
    }

    @Override
    protected void initialize() {
        _cycle = 0;
        _lights.initialize();
    }

    @Override
    protected void execute() {
        _cycle++;
        _lights.setColors(_cycle);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }
}
