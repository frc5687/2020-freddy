package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import org.frc5687.freddy2020.robot.Robot;

public class KillAll extends OutliersCommand {
    private boolean _finished;
    private Robot _robot;

    public KillAll(Robot robot) {
        requires(robot.getArm());
        requires(robot.getElevator());
        requires(robot.getDriveTrain());
        requires(robot.getStilt());
        requires(robot.getCargoIntake());
        requires(robot.getHatchIntake());

        _robot = robot;
    }

    @Override
    protected void initialize() {
        _robot.getCargoIntake().stopRoller();
        _robot.getArm().enableCoastMode();
        _robot.getDriveTrain().enableBrakeMode();

        _robot.getLimelight().disableLEDs();
        _finished = true;
        error("Initialize KillAll Command");
    }

    @Override
    protected void end() {
        error("Ending KillAll Command");
    }

    @Override
    protected boolean isFinished() {
        return _finished;
    }
}
