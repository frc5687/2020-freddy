package org.frc5687.freddy2020.robot.commands;

import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import jaci.pathfinder.Pathfinder;
import jaci.pathfinder.Trajectory;
import jaci.pathfinder.Waypoint;
import jaci.pathfinder.followers.DistanceFollower;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.subsystems.Elevator;
import org.frc5687.freddy2020.robot.OI;

import static org.frc5687.freddy2020.robot.Constants.Elevator.*;
import static org.frc5687.freddy2020.robot.utils.Helpers.limit;

public class MoveElevatorToSetPoint extends OutliersCommand {

    private Elevator _elevator;
    private Elevator.Setpoint _setpoint;
    private Elevator.MotionMode _mode;
    private RampingState _rampingState = RampingState.Done;
    private double _position = 0;

    private double _pidOutput;
    private double _pathOutput;

    private OI _oi;

    private PIDController _pidController;

    private Trajectory _trajectory;
    private DistanceFollower _pathFollower;
    private Notifier _pathNotifier;
    private Notifier _mainNotifier;
    private long _startTime;
    private double _step;
    private int _rampDirection = 0;
    private double _rampMid = 0;
    private long _creepEndTime = 0;
    private double _speed;
    private boolean _topTriggered = false;
    private boolean _bottomTriggered = false;

    private double _ticksPerStep;

    public MoveElevatorToSetPoint(Elevator elevator, Elevator.Setpoint setpoint, Elevator.MotionMode mode, OI oi, double speed) {
        _elevator = elevator;
        requires(_elevator);
        _setpoint = setpoint;
        _mode = mode;
        _speed = speed;

        _oi = oi;

        _pidController = new PIDController(PID.kP, PID.kI, PID.kD, _elevator, new PIDListener());
        _pidController.setAbsoluteTolerance(TOLERANCE);
        _pidController.setOutputRange(-MAX_SPEED_DOWN, MAX_SPEED_UP);
        _pidController.setInputRange(Elevator.Setpoint.Bottom.getValue(), Elevator.Setpoint.Top.getValue());
        _pidController.disable();

        _mainNotifier = new Notifier(this::poll);

        // logMetrics("Ramp/Mode", "Setpoint", "Position", "Ramp/Step", "Ramp/RawSpeed", "Ramp/RampedSpeed", "TopHall", "BottomHall");
    }

    @Override
    protected void initialize() {
        super.initialize();

        _topTriggered = _elevator.isHallEffectTriggered(Elevator.HallEffectSensor.TOP);
        _bottomTriggered = _elevator.isHallEffectTriggered(Elevator.HallEffectSensor.BOTTOM);

        _ticksPerStep = Robot.pickConstant(TICKS_PER_STEP_COMP, TICKS_PER_STEP_PROTO);

        if (_mode== Elevator.MotionMode.Simple &&  _elevator.getPosition() > Elevator.Setpoint.WarningZone.getValue()) {
            _mode = Elevator.MotionMode.Ramp;
        }
        _step = 0;
        _position = _elevator.getPosition();

        if (_setpoint== Elevator.Setpoint.ClearBumper && _position > Elevator.Setpoint.ClearBumper.getValue()) {
            error("Skipping setpoint " + _setpoint.name() + "  b/c height is " + _position);
            return;
        }

        if (withinTolerance()) { return; }
        error("Moving to setpoint " + _setpoint.name() + " (" + _setpoint.getValue() + ") using " + _mode.name() + " mode.");
        switch(_mode) {
            case Simple:
                _rampDirection = (int)Math.copySign(1, _setpoint.getValue() - _position);
                break;
            case PID:
                _pidController.setSetpoint(_setpoint.getValue());
                _pidController.enable();
                break;
            case Path:
                _trajectory = getTrajectory((long)_elevator.getPosition(), _setpoint.getValue());
                _pathFollower = new DistanceFollower(_trajectory);
                _pathFollower.configurePIDVA(Path.kP, Path.kI, Path.kD, 1/MAX_VELOCITY_IPS, 0);

                _pathNotifier = new Notifier(this::followPath);
                _pathNotifier.startPeriodic(_trajectory.get(0).dt);
                break;
            case Ramp:
                _rampingState = RampingState.RampUp;
                double startPosition = _elevator.getPosition();
                if (_setpoint.getValue() > startPosition) {
                    _rampDirection = +1;
                    _rampMid = startPosition + (_setpoint.getValue() - startPosition)/2;
                } else if (_setpoint.getValue() < _elevator.getPosition()) {
                    _rampDirection = -1;
                    _rampMid = startPosition + (_setpoint.getValue() - startPosition)/2;
                } else if (_setpoint.getHall() == Elevator.HallEffectSensor.BOTTOM && !_elevator.isAtBottom()){
                    error("BottomCreep");
                    _rampingState = RampingState.Creep;
                    _creepEndTime = System.currentTimeMillis() + CREEP_TIME;
                    _rampDirection = -1;
                } else if (_setpoint.getHall() == Elevator.HallEffectSensor.TOP && !_elevator.isAtTop()){
                    error("TopCreep");
                    _rampingState = RampingState.Creep;
                    _creepEndTime = System.currentTimeMillis() + CREEP_TIME;
                    _rampDirection = 1;
                } else {
                    _rampDirection = 0;
                }
                break;
        }
        _startTime = System.currentTimeMillis();
        _mainNotifier.startPeriodic(.01);
    }

    protected void poll() {
        if (this.isRunning()) {
            _topTriggered |= _elevator.isHallEffectTriggered(Elevator.HallEffectSensor.TOP);
            _bottomTriggered |= _elevator.isHallEffectTriggered(Elevator.HallEffectSensor.BOTTOM);
            if (_rampDirection > 0 && _topTriggered) {
                _elevator.setSpeed(0);
                _rampDirection = 0;
            } else if (_rampDirection < 0 && _bottomTriggered) {
                _elevator.setSpeed(0);
                _rampDirection = 0;
            }
        }
    }

    @Override
    protected void execute() {
        _step++;
        super.execute();
        double speed;

        if (_setpoint== Elevator.Setpoint.ClearBumper && _position > Elevator.Setpoint.ClearBumper.getValue()) {
            error("Ending setpoint " + _setpoint.name() + "  b/c height is " + _position);
            return;
        }

        _position = _elevator.getPosition();

        metric("Position", _position);
        metric("Setpoint", _setpoint.getValue());
        metric("TopHall", _elevator.isAtTop());
        metric("BottomHall", _elevator.isAtBottom());
        metric("TopTriggered", _topTriggered);
        metric("BottomTriggered", _bottomTriggered);

        switch(_mode) {
            case Simple:

                if (_position  < _setpoint.getValue() - TOLERANCE) {
                    _elevator.setSpeed(_speed == 0 ? SPEED_UP : _speed, false, true);
                } else if (_position > _setpoint.getValue() + TOLERANCE) {
                    _elevator.setSpeed(_speed == 0 ? -SPEED_DOWN : -_speed, false, true);
                } else {
                    _elevator.setSpeed(0);
                }
                break;
            case PID:
                _elevator.setSpeed(_pidOutput, true);
                break;
            case Path:
                _elevator.setSpeed(_pathOutput);
                break;
            case Ramp:
                _elevator.setSpeed(getRampedSpeed());
                metric("Ramp/Mode", _rampingState.name());
                break;
            default:
                _elevator.setSpeed(0);
                break;
        }
    }

    private double getRampedSpeed() {
        double speed = 0;
        double goalSpeed = 0;
        double minSpeed = MIN_SPEED;

        if (_rampDirection > 0) {
            if (_rampingState==RampingState.Recenter) {
                if (_position < _setpoint.getValue() + TOLERANCE) {
                    _rampingState = RampingState.Done;
                    return 0;
                }
                return -MIN_SPEED;
            }

            goalSpeed = SPEED_UP;
            if (_elevator.isAtTop() || _topTriggered) {
                _rampingState = RampingState.Done;
                metric("Ramp/RampedSpeed", 0);
                return 0;
            } else if (_position >= _setpoint.getValue() - TOLERANCE) {
                if (_setpoint.getHall()== Elevator.HallEffectSensor.TOP && !_elevator.isAtTop() && !_topTriggered) {
                    if (_rampingState != RampingState.Creep) {
                        _rampingState = RampingState.Creep;
                        _creepEndTime = System.currentTimeMillis() + CREEP_TIME;
                    }
                    metric("Ramp/RampedSpeed", minSpeed);
                } else if (_position >= _setpoint.getValue() + TOLERANCE) {
                    _rampingState = RampingState.Recenter;
                    return -MIN_SPEED;
                } else {
                    _rampingState = RampingState.Done;
                    metric("Ramp/RampedSpeed", 0);
                    return 0;
                }
            }
        } else if (_rampDirection < 0) {
            if (_rampingState==RampingState.Recenter) {
                if (_position > _setpoint.getValue() - TOLERANCE) {
                    _rampingState = RampingState.Done;
                    return 0;
                }
                return MIN_SPEED;
            }

            goalSpeed = SPEED_DOWN;
            if (_elevator.isAtBottom() || _bottomTriggered) {
                _rampingState = RampingState.Done;
                metric("Ramp/RampedSpeed", 0);
                return 0;
            } else if (_position <= _setpoint.getValue() + TOLERANCE) {
                if (_setpoint.getHall()== Elevator.HallEffectSensor.BOTTOM && !_elevator.isAtBottom() && !_bottomTriggered) {
                    if (_rampingState!=RampingState.Creep) {
                        _rampingState = RampingState.Creep;
                        _creepEndTime = System.currentTimeMillis() + CREEP_TIME;
                    }
                    metric("Ramp/RampedSpeed", minSpeed);
                } else if (_position < _setpoint.getValue() - TOLERANCE) {
                    _rampingState = RampingState.Recenter;
                    return MIN_SPEED;
                } else {
                    _rampingState = RampingState.Done;
                    metric("Ramp/RampedSpeed", 0);
                    return 0;
                }
            }
        }
        speed = goalSpeed;

        metric("Ramp/RawSpeed", speed);
        metric("Ramp/Mode", _rampingState.name());
        metric("Ramp/Step", _step);

        switch(_rampingState) {
            case RampUp:
                speed = minSpeed + _step * ((goalSpeed - minSpeed) / STEPS_UP);
                if (_rampDirection>0 && _position >= _rampMid
                        || _rampDirection < 0 && _position <= _rampMid) {
                    // Halfway there . . . switch to ramping down
                    _step = 0;
                    _rampingState = RampingState.RampDown;
                    error("Elevator ramping state " +_rampingState);
                } else if(Math.abs(_setpoint.getValue() - _elevator.getPosition()) <=  _ticksPerStep * STEPS_DOWN) {
                    // We've reached the slow-down range
                    _step = 0;
                    _rampingState = RampingState.RampDown;
                    error("Elevator ramping state " +_rampingState);
                } else if (_step >= STEPS_UP) {
                    // Fully ramped up--switch to steady-state
                    _rampingState = RampingState.Steady;
                    error("Elevator ramping state " +_rampingState);
                }
                break;
            case Steady:
                speed = goalSpeed;
                if(Math.abs(_setpoint.getValue() - _elevator.getPosition()) <=  _ticksPerStep * STEPS_DOWN) {
                    _step = 0;
                    _rampingState = RampingState.RampDown;
                    error("Elevator ramping state " +_rampingState);
                }
                break;
            case RampDown:
                //DriverStation.reportError("" + (minSpeed + (STEPS_DOWN - _step) * ((speed - minSpeed) / STEPS_DOWN)) + " = " + minSpeed + " + ( " + STEPS_DOWN + " - " + _step + ") * (( " + speed + " - " + minSpeed + ") / " + STEPS_DOWN+ ")", false );
                speed = minSpeed + (STEPS_DOWN - _step) * ((goalSpeed - minSpeed) / STEPS_DOWN);
                //speed = (Constants.Elevator.GOAL_SPEED -(_step/Constants.Elevator.STEPS))* (Constants.Elevator.GOAL_SPEED - Constants.Elevator.MIN_SPEED);
                //if (_elevator.getRawMAGEncoder() == _setpoint.getValue()) {
                //}
                break;
            case Creep:
                speed = minSpeed;
                if (System.currentTimeMillis() >= _creepEndTime) {
                    _rampingState = RampingState.Done;
                }
                break;
            case Done:
                speed = 0;
                break;
        }
        speed = limit(speed, minSpeed, goalSpeed);
        speed = speed * _rampDirection;

        metric("Ramp/RampedSpeed", speed);

        return speed;
    }
    private boolean withinTolerance() {
        return Math.abs(_position-_setpoint.getValue()) <= TOLERANCE;
    }
    @Override
    protected boolean isFinished() {
        if (_setpoint== Elevator.Setpoint.ClearBumper && _position > Elevator.Setpoint.ClearBumper.getValue()) {
            error("Skipping setpoint " + _setpoint.name() + "  b/c height is " + _position);
            return true;
        }

        if (withinTolerance()) {
            return true;
        }
        switch (_mode) {
            case PID:
                return _pidController.onTarget();
            case Path:
                return _pathFollower.isFinished();
            case Ramp:
                return _rampingState == RampingState.Done;
            case Simple:
                if (_rampDirection > 0) {
                    return _position > _setpoint.getValue();
                } else {
                    return _position < _setpoint.getValue();
                }
        }
        return false;
    }

    @Override
    protected void end() {
        long endTime = System.currentTimeMillis();

        // Resetting members used by the notifier.
        _topTriggered = false;
        _bottomTriggered = false;
        _rampDirection = 0;

        // DriverStation.reportError("MoveElevatorToSetpoint Ran for " + (endTime - _startTime) + " millis, stopped at " + _position + ", state=" + _rampingState.name(), false);
        if (_mainNotifier!=null) {
            _mainNotifier.stop();
        }
        if (_pidController!=null) {
            _pidController.disable();
        }
        if (_pathNotifier!=null) {
            _pathNotifier.stop();
        }
        _elevator.setSpeed(0);

        if (_oi!=null) {
            switch (_setpoint) {
                case Bottom:
                case Hatch1:
                case Port1:
                case StartHatch:
                    _oi.pulseOperator(1);
                    break;
                case Hatch2:
                case Port2:
                    _oi.pulseOperator(2);
                    break;
                case Top:
                case Hatch3:
                case Port3:
                    _oi.pulseOperator(3);
                    break;
            }
        }

        info("Reached setpoint " + _setpoint.name() + " (" + _position + ")");
        super.end();
    }

    private class PIDListener implements PIDOutput {

        public double get() {
            return _pidOutput;
        }

        @Override
        public void pidWrite(double output) {
            _pidOutput = output;
        }

    }

    private void followPath() {
        if (_pathFollower.isFinished()) {
            _pathNotifier.stop();
        } else {
            double _speed = _pathFollower.calculate(_elevator.getPosition());
            _pathOutput = _speed;
        }
    }


    private Trajectory getTrajectory(long startPosition, long endPosition) {
        double startInches = startPosition/Constants.Elevator.TICKS_PER_INCH;
        double endInches = endPosition/Constants.Elevator.TICKS_PER_INCH;

        Waypoint[] points = new Waypoint[] {
                new Waypoint(0, startInches, 0),      // Waypoint @ x=-4, y=-1, exit angle=-45 degrees
                new Waypoint(0, endInches, 0),                        // Waypoint @ x=-2, y=-2, exit angle=0 radians
        };

        Trajectory.Config config = new Trajectory.Config(Trajectory.FitMethod.HERMITE_CUBIC, Trajectory.Config.SAMPLES_FAST, 0.02, MAX_VELOCITY_IPS, MAX_VELOCITY_IPS, 60.0);
        Trajectory trajectory = Pathfinder.generate(points, config);

        return trajectory;
    }


    private enum RampingState {
        RampUp(0),
        Steady(1),
        RampDown(2),
        Creep(3),
        Recenter(4),
        Done(5);

        private int _value;

        RampingState(int value) { this._value = value; }

        public int getValue() { return _value; }
    }

}
