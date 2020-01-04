package org.frc5687.freddy2020.robot.subsystems;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Spark;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.RobotMap;
import org.frc5687.freddy2020.robot.commands.DriveLights;

public class Lights extends OutliersSubsystem {


    private Robot _robot;
    private Spark _right;
    private Spark _left;
    private DriverStation.Alliance _alliance;

    private double _mainColor;
    private int _blink;

    public Lights(Robot robot) {
        _robot = robot;
        _right = new Spark(RobotMap.PWM.LeftBlinkin);
        _left = new Spark(RobotMap.PWM.RightBlinkin);
    }

    public void initialize() {
        _alliance = DriverStation.getInstance().getAlliance();
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveLights(_robot.getLights()));
    }

    public void setRight(double val) {
        // DriverStation.reportError("Setting left to " + val, false);
        _right.set(val);
    }
    public void setLeft(double val) {
        // DriverStation.reportError("Setting left to " + val, false);
        _left.set(val);
    }
    public void setColor(double color, int blink) {
        _mainColor = color;
        _blink = blink;
    }

    public void setColors(long cycle) {
        if (_blink==0) {
            setRight(_mainColor);
            setLeft(_mainColor);
        } else if ((cycle % _blink) == 0) {
            setRight(_mainColor);
            setLeft(_mainColor);
        } else {
            setRight(0);
            setLeft(0);
        }
    }

    @Override
    public void updateDashboard() {
        metric("MainColor", _mainColor);
        metric("Blink", _blink);
    }

}
