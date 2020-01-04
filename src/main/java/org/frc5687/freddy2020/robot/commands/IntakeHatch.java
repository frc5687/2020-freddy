package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.command.CommandGroup;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.commands.intake.*;
import org.frc5687.freddy2020.robot.subsystems.Elevator;

public class IntakeHatch extends CommandGroup {
     public IntakeHatch(Robot robot) {
         addSequential(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Bottom, Elevator.MotionMode.Ramp, null, 0.0));
         addSequential(new PointClaw(robot.getHatchIntake()));
         addSequential(new CargoIntakeUp(robot.getCargoIntake()));
         addSequential(new ClawWristUp(robot));
     }
}
