package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.subsystems.Elevator;

public class DriveElevator extends OutliersCommand{
    
    private Elevator _elevator;
    private OI _oi;

    public DriveElevator(Robot robot, Elevator elevator) {
        _elevator = elevator;
        _oi = robot.getOI();
        requires(_elevator);
    }

    @Override
    protected void initialize() {
    }

    @Override
    public void execute() {
        // Read speed stick positions from OI
        double speed = _oi.getElevatorSpeed();
        // Send to the Elevator
        _elevator.setSpeed(speed);
    }


    @Override
    protected boolean isFinished() {
        return false;
    }

    @Override
    protected void end() {
        // Set Elevator motor speeds to 0 and set break mode?s
    }
}

