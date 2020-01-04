package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.intake.CargoIntakeUp;
import org.frc5687.freddy2020.robot.commands.intake.ClawWristUp;
import org.frc5687.freddy2020.robot.commands.intake.GripClaw;
import org.frc5687.freddy2020.robot.subsystems.Elevator;

public class SandstormPickup extends CommandGroup {
    public SandstormPickup(Robot robot) {
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Simple, null, Constants.Elevator.SANDSTORM_PICKUP_SPEED));
        addSequential(new GripClaw(robot.getHatchIntake(), Constants.Intake.CLOSE_CLAW_MILLI_SS));
        addSequential(new CargoIntakeUp(robot.getCargoIntake()));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.ClearBumper, Elevator.MotionMode.Simple, null, Constants.Elevator.SANDSTORM_PICKUP_SPEED));
        addSequential(new ClawWristUp(robot));
        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Simple, null, Constants.Elevator.MODE_SPEED));
        addSequential(new SetConfiguration(robot, Robot.Configuration.hatch));
    }

}
 