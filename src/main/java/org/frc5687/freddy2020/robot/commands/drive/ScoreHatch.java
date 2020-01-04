package org.frc5687.freddy2020.robot.commands.drive;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.AutoDrive;
import org.frc5687.freddy2020.robot.commands.AutoDriveToTarget;
import org.frc5687.freddy2020.robot.commands.HatchMode;
import org.frc5687.freddy2020.robot.commands.intake.GripClaw;
import org.frc5687.freddy2020.robot.commands.intake.PointClaw;

public class ScoreHatch extends CommandGroup {
    public ScoreHatch(Robot robot) {
        addSequential(new GripClaw(robot.getHatchIntake()));
        addSequential(new AutoDriveToTarget(robot, Constants.Auto.DriveToTarget.MAX_SPEED, Constants.Auto.DriveToTarget.STOP_DISTANCE, Constants.Auto.DriveToTarget.DISTANCE_TOLERANCE, "Score"));
        addSequential(new PointClaw(robot.getHatchIntake()));
        addSequential(new AutoDrive(robot.getDriveTrain(), robot.getIMU(),robot.getHatchIntake(), robot.getElevator(), -12.00, Constants.Auto.DriveToTarget.MAX_SPEED, true, true, 1000, "Retreat", 1000));
    }
}
