package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.drive.AutoDriveToTargetSimple;
import org.frc5687.freddy2020.robot.commands.intake.PointClaw;

public class AutoRocket extends CommandGroup {
    public AutoRocket (Robot robot, boolean left) {
        addSequential(new SandstormPickup(robot));
        addSequential(new AutoDrive (robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(),60,0.75, false, false, left?-30.0 : 30.0, "Left Rocket", 5000));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), 0.6, false, 0, false));
        addSequential(new PointClaw(robot.getHatchIntake()));
    }
}
