package org.frc5687.freddy2020.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import jaci.pathfinder.Trajectory;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.*;
import org.frc5687.freddy2020.robot.commands.intake.GripClaw;
import org.frc5687.freddy2020.robot.commands.intake.PointClaw;
import org.frc5687.freddy2020.robot.subsystems.Shifter;

import java.io.IOException;

public class TwoHatchCloseAndFarRocket extends CommandGroup {
    public TwoHatchCloseAndFarRocket (Robot robot, boolean OffHAB, boolean left) throws IOException {
        if (OffHAB) {
            addSequential(new AutoLaunch(robot));
        }
        addParallel(new SandstormPickup(robot));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -30 : 30, .7, 500, 2, "aligning to rocket"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, true));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -4, .7, false, true, 0, "reverse 4 inches", 200));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -175 : 175, .7, 1000, 2, "aligning to loading station"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, false));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -179 : 179, .7, 250, 5, "aligning to rocket"));
        addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), robot.getLimelight(), robot.getPoseTracker(), left ? "LeftFarRocket" : "RightFarRocket", 0, true));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, false, 3));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -3.5, .7, false, true, 0, "reverse 12 inches", 180));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? 90 : -90, .7, 1000, 5, "aligning to rocket"));
        addSequential(new CargoMode(robot));
        addSequential(new AutoIntake(robot));
    }

}
