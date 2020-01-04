package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.intake.*;
import org.frc5687.freddy2020.robot.subsystems.Elevator;

public class IntakeCargo extends CommandGroup {

    public IntakeCargo(Robot robot) {
//        addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Simple, null, Constants.Elevator.MODE_SPEED));
        addSequential(new StartRoller(robot.getCargoIntake(), true));
        addSequential(new StopRoller(robot.getCargoIntake()));
        addSequential(new CargoIntakeUp(robot.getCargoIntake()));
    }
}
