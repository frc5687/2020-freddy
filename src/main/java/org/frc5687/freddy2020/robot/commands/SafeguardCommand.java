package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import org.frc5687.freddy2020.robot.Robot;

public class SafeguardCommand extends ConditionalCommand {
    private Robot _robot;
    private long _timeLimit;


    public SafeguardCommand(Robot robot, Command command, long timeLimit) {
        super(command);
        _robot = robot;
        _timeLimit = timeLimit;
    }

    @Override
    protected boolean condition() {
//        if (DriverStation.getInstance().isFMSAttached()) {
        double timeLeft = DriverStation.getInstance().getMatchTime();
        if (DriverStation.getInstance().isOperatorControl() && _timeLimit < 0) {
            // Only trigger in last _timeLimit seconds...
            if (timeLeft <= -_timeLimit) {
                return true;
            } else {
                DriverStation.reportError("Skipping command with " + timeLeft + " left when " + _timeLimit + " passed", false);
            }
        } else {
            DriverStation.reportError("Skipping command because not in teleop when " + _timeLimit + " passed", false);
        }
        return false;
    }
}
