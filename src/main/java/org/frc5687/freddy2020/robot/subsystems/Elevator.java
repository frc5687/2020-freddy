package org.frc5687.freddy2020.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.RobotMap;
import org.frc5687.freddy2020.robot.commands.DriveElevator;
import org.frc5687.freddy2020.robot.utils.HallEffect;
import static org.frc5687.freddy2020.robot.utils.Helpers.*;
import static org.frc5687.freddy2020.robot.Constants.Elevator.*;

public class Elevator extends OutliersSubsystem implements PIDSource {

    private CANSparkMax _elevator;
    private Encoder _elevatorEncoder;
    private CANEncoder _neoElevatorEncoder;
    private Robot _robot;

    private HallEffect _topHall;
    private HallEffect _bottomHall;

    private double _offset = 0;
    private OI _oi;

    public Elevator(Robot robot) {
        _robot = robot;
        _oi = robot.getOI();

        try {
            _elevator = new CANSparkMax(RobotMap.CAN.SPARKMAX.ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
            _elevator.setInverted(Constants.Elevator.ELEVATOR_MOTOR_INVERTED);
            _neoElevatorEncoder = _elevator.getEncoder();
        } catch (Exception e) {
            error("Unable to allocate elevator controller: " + e.getMessage());
        }
        _elevatorEncoder = new Encoder(RobotMap.DIO.ELEVATOR_B, RobotMap.DIO.ELEVATOR_A);

        _topHall = new HallEffect(RobotMap.DIO.ELEVATOR_TOP_HALL);
        _bottomHall = new HallEffect(RobotMap.DIO.ELEVATOR_BOTTOM_HALL);

    }
    public void setSpeed(double speed) {
        setSpeed(speed, false, false);
    }
    public void setSpeed(double speed, boolean overrideRamp) { setSpeed(speed,overrideRamp, false); }
    public void setSpeed(double speed, boolean overrideRamp, boolean overrideJello) {
        metric("RequestedSpeed", speed);
        speed = limit(speed, -MAX_SPEED_DOWN, MAX_SPEED_UP);
        if (_oi.isOverridePressed()) {
            overrideJello = true;
            overrideRamp = true;
        }

        if (!overrideRamp) {
            if (speed > 0) {
                speed = limit(speed, -MAX_SPEED_DOWN, _elevator.get() + (MAX_SPEED_UP / 5), "ramping up");
            } else if (speed < 0) {
                speed = limit(speed, _elevator.get() - (MAX_SPEED_DOWN / 5), MAX_SPEED_UP, "ramping down");
            }
        }
        if (isAtTop()) {
            speed = limit(speed, -MAX_SPEED_DOWN,  0, "at top");
        } else if (!overrideJello && isNearTop()) {
            speed = limit(speed, -MAX_SPEED_DOWN,  JELLO_SPEED_UP, "near top");
        } else if (isAtBottom()) {
            speed = limit(speed, 0, MAX_SPEED_UP, "at bottom");
        } else if (!overrideJello && isNearBottom()) {
            speed = limit(speed, -JELLO_SPEED_DOWN, MAX_SPEED_UP, "near bottom");
        }
        metric("ElevatorSpeed",speed);

        if (_elevator==null) { return; }
        if (speed>=0 && isAtTop()) {
            resetEncoder(Setpoint.Top.getValue());
        } else if (speed <= 0 && isAtBottom()) {
            resetEncoder(Setpoint.Bottom.getValue());
        }

        if (speed==0 && _elevator.get()!=0){
            DriverStation.reportError("Setting speed to 0 ", true);
        }
        _elevator.set(speed);
    }

    public void enableBrakeMode() {
        if (_elevator==null) { return; }
        _elevator.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableCoastMode() {
        if (_elevator==null) { return; }
        // _elevator.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    private double getRawNeoEncoder() {
        if (_elevator==null) { return 0; }
        return _neoElevatorEncoder.getPosition();
    }

    private double getRawMAGEncoder() {
        return _elevatorEncoder.get();
    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveElevator(_robot, this));
    }
    @Override
    public void updateDashboard() {
        metric("MAGEncoder", getRawMAGEncoder());

//        metric("NEOEncoder", getRawNeoEncoder());
        metric("Position", getPosition());
        metric("Bottom", isAtBottom());
        metric("Top", isAtTop());
    }
    public boolean isAtTop() { return _topHall.get(); }

    public boolean isNearTop() { return isAtTop() || (getPosition() > Setpoint.Top.getValue() - TOP_JELLO_ZONE); }

    public boolean isAtBottom() { return _bottomHall.get(); }

    public boolean isNearBottom() { return isAtBottom() || (getPosition() < Setpoint.Bottom.getValue() + BOTTOM_JELLO_ZONE); }



    public boolean isLimelightClear() {
        return isAtBottom()
                || (getPosition() < Setpoint.Bottom.getValue() + BOTTOM_CAM_ZONE)
                || isNearTop();
    }

    public void resetEncoder() {
        resetEncoder(0);
    }

    public void resetEncoder(long position) {
        _offset = position - getRawMAGEncoder();
//        DriverStation.reportError("Resetting elevator Encoder pos" + position, true);
//        DriverStation.reportError("Resetting elevator Encoder raw" + getRawMAGEncoder(), true);


    }

    public double getPosition() {
        return getRawMAGEncoder() + _offset;
    }

    public boolean isHallEffectTriggered(HallEffectSensor hall) {
        switch(hall) {
            case TOP:
                return isAtTop();
            case BOTTOM:
                return isAtBottom();
        }
        return false;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public double pidGet() {
        return getPosition();
    }

    public boolean isAboveMiddle() {
        return getPosition() >= Setpoint.SlowPoint.getValue();
    }

    public enum HallEffectSensor {
        BOTTOM,
        TOP
    }


    public enum Setpoint {
        Bottom(0, HallEffectSensor.BOTTOM),
        Port1(1, HallEffectSensor.BOTTOM),
        Hatch1(2, HallEffectSensor.BOTTOM),
        Secure(10),
        ClearBumper(300),
        WarningZone(700),
        StartHatch(701, 710),
        HPMode(1230),
        SlowPoint(2000, 1000),
        Port2(2480, 2168),
        Hatch2(2550, 1954),
        Port3(4798, 4098, HallEffectSensor.TOP),
        Hatch3(4799, 4099, HallEffectSensor.TOP),
        Top(4800, 4100, HallEffectSensor.TOP);

        private int _competitionValue;
        private int _practiceValue;
        private HallEffectSensor _hall;

        Setpoint(int value) {
            this(value, value, null);
        }

        Setpoint(int competitionValue, int practiceValue) {
            this(competitionValue, practiceValue, null);

        }

        Setpoint(int value, HallEffectSensor hall) {
            this(value, value, hall);
        }

        Setpoint(int competitionValue, int practiceValue, HallEffectSensor hall) {
            _competitionValue = competitionValue;
            _practiceValue = practiceValue;
            _hall = hall;
        }

        public int getValue() {
            return Robot.identityMode ==Robot.IdentityMode.practice ? _practiceValue : _competitionValue;
        }

        public HallEffectSensor getHall() { return _hall; }
    }

    public enum MotionMode {
        Simple(0),
        PID(1),
        Path(2),
        Ramp(3);

        private int _value;

        MotionMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }


}
