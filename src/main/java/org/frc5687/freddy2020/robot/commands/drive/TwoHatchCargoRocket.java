package org.frc5687.freddy2020.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.*;
import org.frc5687.freddy2020.robot.commands.intake.GripClaw;
import org.frc5687.freddy2020.robot.commands.intake.PointClaw;
import org.frc5687.freddy2020.robot.subsystems.Shifter;

public class TwoHatchCargoRocket extends CommandGroup {
    public TwoHatchCargoRocket(Robot robot, boolean OffHAB, boolean left) {
        if (OffHAB) {
            addSequential(new AutoLaunch(robot));
        }
        addParallel(new SandstormPickup(robot));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, true));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -3, .6, false,true, 0, "reverse 12 inches", 200));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -90 : 90, .8, 500, 2, "aligning to HP Station"));
        addSequential(new AutoDrivePath(robot.getDriveTrain(), robot.getIMU(), robot.getLimelight(), robot.getPoseTracker(), left ? "CargoToLeftLoading" : "CargoToRightLoading", 0, false));

        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .9, false, 0, false));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -12, .7,false, true, 0, "reverse 12 inches", 500));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -170 : 170, .7, 200, 2, "aligning to rocket"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -72, .7,false, true, 0, "reverse 12 inches", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -30 : 30, .7, 1000, 2, "aligning to rocket"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .6, false, 0, false));

        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -12, .7, false,true, 0, "reverse 12 inches", 1000));
    }
}
