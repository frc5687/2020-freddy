package org.frc5687.freddy2020.robot.commands;

import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.subsystems.*;

import static org.frc5687.freddy2020.robot.Constants.Auto.Climb.*;
import static org.frc5687.freddy2020.robot.Constants.Arm.*;

public class AutoClimb extends OutliersCommand {
    private Robot _robot;

    private Stilt _stilt;
    private Arm _arm;
    private DriveTrain _driveTrain;

    private CargoIntake _cargoIntake;
    private HatchIntake _hatchIntake;

    private ClimbState _climbState;
    private long _stiltTimeout = 0;
    private long _parkTimeout = 0;
    private boolean _highHab = true;

    private double _contactAngle;
    private double _slowAngle;
    private double _bottomAngle;

    private OI _oi;

    public AutoClimb(Stilt stilt, Arm arm, DriveTrain driveTrain, CargoIntake cargoIntake, HatchIntake hatchIntake, Robot robot, boolean highHab) {
        _stilt = stilt;
        _arm = arm;
        _driveTrain = driveTrain;

        _cargoIntake = cargoIntake;
        _hatchIntake = hatchIntake;

        _robot = robot;
        _oi = _robot.getOI();
        _highHab = highHab;

        requires(_stilt);
        requires(_arm);
        requires(_driveTrain);
        requires(_cargoIntake);
        requires(_hatchIntake);
    }

    @Override
    protected void initialize() {
        //if (DriverStation.getInstance().isFMSAttached() && DriverStation.getInstance().getMatchTime() > ENDGAME_CUTOFF) {
        //    _climbState = ClimbState.Done;
        //    error("Climb pressed before endgame");
        //}

       _climbState = _arm.encodersZeroed() ? ClimbState.PositionArm : ClimbState.StowArm;
       metric("EncodersZeroed", _arm.encodersZeroed());
       _driveTrain.enableBrakeMode();
       _robot.setConfiguration(Robot.Configuration.climbing);

       if (_highHab) {
           _contactAngle = H3_CONTACT_ANGLE;
           _slowAngle = H3_SLOW_ANGLE;
           _bottomAngle = H3_BOTTOM_ANGLE;
       } else {
           _contactAngle = H2_CONTACT_ANGLE;
           _slowAngle = H2_SLOW_ANGLE;
           _bottomAngle = H2_BOTTOM_ANGLE;
       }
        _arm.enableBrakeMode();
        _stilt.enableBrakeMode();

        metric("ClimbState", _climbState.name());

    }

    @Override
    public void execute() {
        switch (_climbState) {
            case StowArm:
                _cargoIntake.raiseWrist();
                _hatchIntake.pointClaw();
                _hatchIntake.lowerWrist();
                _arm.setSpeed(STOW_SPEED);
                if (_oi.isOverridePressed() || (_arm.isLeftStowed() && _arm.isRightStowed())) {
                    error("Transitioning to " + ClimbState.PositionArm.name());
                    _climbState = ClimbState.PositionArm;
                }
                break;
            case PositionArm:
                _arm.setSpeed(INITIAL_ARM_SPEED);
                if (_arm.getAngle() >= _contactAngle) {
                    error("Transitioning to " + ClimbState.MoveArmAndStilt.name());
                    _climbState = ClimbState.MoveArmAndStilt;
                }
                break;
            case MoveArmAndStilt:
                _stilt.setLifterSpeed(STILT_SPEED);
                double armSpeed =  _arm.getAngle() >= _slowAngle ? ARM_SLOW_SPEED : ARM_SPEED; // Math.cos(Math.toRadians(_arm.getAngle())) * ARM_SPEED_SCALAR;
                if ((_arm.isLow() || _arm.getAngle() >= _bottomAngle)) {
                    error("Stopping arm");
                    _arm.setSpeed(0);
                } else {
                    error("Running arm");
                    _arm.setSpeed(armSpeed);
                }

                if ((_highHab ?_stilt.isExtended() : _stilt.isAtMiddle())) {
                    error("Stopping stilt");
                    _stilt.setLifterSpeed(_highHab ? STILT_HIGH_HOLD_SPEED : STILT_LOW_HOLD_SPEED);
                } else {
                    error("Running stilt");
                    _stilt.setLifterSpeed(STILT_SPEED);
                }

                if (_oi.isWheelieForwardPressed() ||
                        ((_arm.isLow() || _arm.getAngle() >= _bottomAngle)
                        && (_highHab ?_stilt.isExtended() : _stilt.isAtMiddle()))) {
                    metric("StiltSpeed", 0);
                    metric("ArmSpeed", 0);
                    _arm.setSpeed(0);
                    _driveTrain.disableBrakeMode();
                    error("Transitioning to " + ClimbState.WheelieForward.name());
                    _climbState = ClimbState.WheelieForward;
                }
                metric("StiltSpeed", STILT_SPEED);
                metric("ArmSpeed", armSpeed);
                break;
            case WheelieForward:
                _stilt.setLifterSpeed(_highHab ? STILT_HIGH_HOLD_SPEED : STILT_LOW_HOLD_SPEED);
                _stilt.setWheelieSpeed(_highHab ? WHEELIE_FORWARD_SPEED_HIGH : WHEELIE_FORWARD_SPEED_LOW);
                _driveTrain.disableBrakeMode();
                _driveTrain.cheesyDrive(DRIVE_FORWARD_SPEED,0, false, false);
                metric("WheelieSpeed", WHEELIE_FORWARD_SPEED_HIGH);
                metric("DriveSpeed", DRIVE_FORWARD_SPEED);
                metric("StiltSpeed", STILT_HIGH_HOLD_SPEED);
                if (_highHab ? _stilt.isOnSurface() : _stilt.isOnSurfaceLow()) {
                    metric("WheelieSpeed", 0);
                    metric("DriveSpeed", 0);
                    metric("StiltSpeed", 0);
                    _arm.enableCoastMode();
                    _stilt.setWheelieSpeed(0);
                    error("Transitioning to " + ClimbState.LiftArm.name());
                    _climbState = ClimbState.LiftArm;
                }
                break;
            case LiftArm:
                _arm.setSpeed(RAISE_ARM_SPEED);
                _driveTrain.cheesyDrive(0,0, false, false);
                if (_arm.getAngle() <= ARM_RETRACT_ANGLE) {
                    _climbState = ClimbState.LiftStilt;
                    error("Transitioning to " + ClimbState.LiftStilt.name());
                    _stiltTimeout = System.currentTimeMillis() + (_highHab ? STILT_TIMEOUT_HIGH : STILT_TIMEOUT_LOW);
                }
                break;

            case LiftStilt:
                if (_arm.isStowed()) {
                    _arm.setSpeed(0);
                }
                _stilt.setLifterSpeed(RAISE_STILT_SPEED);
                _driveTrain.enableBrakeMode();
                _driveTrain.cheesyDrive(0,0, false, false);
                metric("StiltSpeed", RAISE_STILT_SPEED);
                if (_stilt.isRetracted()) {
                    _stilt.setLifterSpeed(0);
                    _stilt.enableCoastMode();
                    _driveTrain.resetDriveEncoders();
                    _driveTrain.enableBrakeMode();
                    error("Transitioning to " + ClimbState.Park.name());
                    _parkTimeout = System.currentTimeMillis() + PARK_TIMEOUT;
                    _climbState = ClimbState.Park;
                }
                if (System.currentTimeMillis() >= _stiltTimeout) {
                    _stilt.setLifterSpeed(0);
                    _stilt.enableBrakeMode();
                    error("Transitioning to " + ClimbState.WaitStilt.name());
                    _climbState = ClimbState.WaitStilt;
                }
                break;
            case WaitStilt:
                if (_arm.isStowed()) {
                    _arm.setSpeed(0);
                }
                _stilt.setLifterSpeed(0);
                _driveTrain.cheesyDrive(0,0, false, false);
                metric("StiltSpeed", 0);
                if (_stilt.isRetracted()) {
                    _stilt.enableCoastMode();
                    _driveTrain.resetDriveEncoders();
                    _driveTrain.enableBrakeMode();
                    _climbState = ClimbState.Park;
                    _parkTimeout = System.currentTimeMillis() + PARK_TIMEOUT;
                    error("Transitioning to " + ClimbState.Park.name());
                }
                break;

            case Park:
                if (_arm.isStowed()) {
                    _arm.setSpeed(0);
                }
                metric("DriveSpeed", PARK_SPEED);
                _driveTrain.cheesyDrive(PARK_SPEED, 0, false, false);
                if (_driveTrain.getDistance() > PARK_DISTANCE || System.currentTimeMillis() >= _parkTimeout) {
                    metric("DriveSpeed", 0);
                    _driveTrain.cheesyDrive(0.0,0, false, false);
                    _driveTrain.enableBrakeMode();
                    error("Transitioning to " + ClimbState.Tilt.name());
                    _climbState = ClimbState.Tilt;
                }
                break;
            case Tilt:
                _stilt.setLifterSpeed(STILT_TILT_SPEED);
                if (_stilt.isTilted() || _robot.getIMU().getPitch() >= TILT_PITCH) {
                    error("Transitioning to " + ClimbState.Done.name());
                    _climbState = ClimbState.Done;
                    _stilt.setLifterSpeed(STILT_TILT_HOLD_SPEED);
                    _stilt.enableBrakeMode();
                    error("Transitioning to " + ClimbState.Done.name());
                    _climbState = ClimbState.Done;
                }
                break;
            case Done:
                _robot.setConfiguration(Robot.Configuration.parked);
                _driveTrain.enableBrakeMode();
                _stilt.setLifterSpeed(0);
                _arm.setSpeed(0);
                break;
        }
        metric("ClimbState", _climbState.name());
    }

    @Override
    protected void end() {
        // _stilt.enableCoastMode();
        // _arm.enableCoastMode();
    }

    @Override
    protected boolean isFinished() {
        return _climbState==ClimbState.Done;
    }

    enum ClimbState {
        StowArm(0),
        PositionArm(1),
        MoveArmAndStilt(2),
        WheelieForward(3),
        LiftArm(4),
        LiftStilt(5),
        WaitStilt(6),
        Park(7),
        Tilt(8),
        Done(9);

        private int _value;

        ClimbState(int value) { this._value = value; }

        public int getValue() { return _value; }
    }
}
