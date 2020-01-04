package org.frc5687.freddy2020.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.AutoAlign;
import org.frc5687.freddy2020.robot.commands.AutoDrive;
import org.frc5687.freddy2020.robot.commands.AutoLaunch;
import org.frc5687.freddy2020.robot.commands.SandstormPickup;
import org.frc5687.freddy2020.robot.commands.intake.GripClaw;
import org.frc5687.freddy2020.robot.commands.intake.PointClaw;

public class TwoHatchCargoShip extends CommandGroup {
    public TwoHatchCargoShip(Robot robot, boolean OffHAB, boolean left) {
        if(OffHAB){
            addSequential(new AutoLaunch(robot));
        }
        addParallel(new SandstormPickup(robot));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(),robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .7, false, 0, true));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -8, .5, false, true, 0, "Backing away from CargoShip", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? 90 : -90, 1,1000, 2,"Turning to back up near Human Player Station"));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -170 : 170, 1,1000, 2,"Aligning to Human Player Station"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(), robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .7,false, 0, false));
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -12, .5, false, true, 0, "Backing away from CargoShip", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? -170 : 170, 1,1000, 2,"Aligning to Human Player Station"));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -100, .5, false, true, 0, "Backing away from CargoShip", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), left ? 90 : -90, 1,1000, 2,"Turning to back up near Human Player Station"));
        addSequential(new AutoDriveToTargetSimple(robot.getDriveTrain(), robot.getIMU(), robot.getOI(),robot.getLimelight(), robot.getElevator(), robot.getCargoIntake(), robot.getHatchIntake(), robot.getPoseTracker(), .7, false, 0, false));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(), robot.getHatchIntake(), robot.getElevator(), -12, .5, false, true, 0, "Backing away from CargoShip", 1000));
        addSequential(new AutoAlign(robot.getDriveTrain(), robot.getIMU(), -180, 1,1000, 2,"Aligning to Human Player Station"));
    }
}
