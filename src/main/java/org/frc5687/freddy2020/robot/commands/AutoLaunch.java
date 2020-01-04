package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.freddy2020.robot.Robot;

public class AutoLaunch extends CommandGroup {
    public AutoLaunch(Robot robot) {
        addSequential(new AutoDrive(robot.getDriveTrain(),robot.getIMU(), robot.getHatchIntake(), robot.getElevator(),24,.5,false, true, 0, "", 2000));
//        addSequential(new SandstormPickup(robot));
    }
}
