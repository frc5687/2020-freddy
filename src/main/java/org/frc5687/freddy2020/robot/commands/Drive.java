package org.frc5687.freddy2020.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;
import org.frc5687.freddy2020.robot.subsystems.DriveTrain;
import org.frc5687.freddy2020.robot.subsystems.Elevator;
import org.frc5687.freddy2020.robot.subsystems.HatchIntake;
import org.frc5687.freddy2020.robot.utils.BasicPose;
import org.frc5687.freddy2020.robot.utils.Helpers;
import org.frc5687.freddy2020.robot.utils.Limelight;

import org.frc5687.freddy2020.robot.utils.PoseTracker;

import static org.frc5687.freddy2020.robot.Constants.Auto.Align.STEER_K;

public class Drive extends OutliersCommand {

    private OI _oi;
    private DriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;
    private PoseTracker _poseTracker;
    private Elevator _elevator;
    private HatchIntake _hatchIntake;
    private CargoIntake _cargoIntake;

    private PIDController _angleController;

    private double _anglePIDOut;
    private double _angle;
    private double _turnSpeed;
    private boolean _targetSighted;
    private long _lockEnd;
    private DriveState _driveState = DriveState.normal;

    private long _seekMax;
    private double _stickyLimit;
    private boolean _lockout = false;

    private double _mediumZone;
    private double _slowZone;

    private double _slowSpeed;
    private double _mediumSpeed;

    private int garbageCount = 0;

    public Drive(DriveTrain driveTrain, AHRS imu, OI oi, Limelight limelight, Elevator elevator, CargoIntake cargoIntake,HatchIntake hatchIntake, PoseTracker poseTracker) {
        _driveTrain = driveTrain;
        _oi = oi;
        _imu = imu;
        _limelight = limelight;
        _elevator = elevator;
        _poseTracker = poseTracker;
        _cargoIntake = cargoIntake;
        _hatchIntake = hatchIntake;
        requires(_driveTrain);

//        logMetrics("State", "StickSpeed", "StickRotation", "LeftPower", "RightPower", "LeftMasterAmps", "LeftFollowerAmps", "RightMasterAmps", "RightFollowerAmps", "TurnSpeed", "Pose","Yaw","PoseAngle","LimelightAngle","TargetAngle", "TargetDistance", "Pipeline", "Lockout");
    }



    @Override
    protected void initialize() {
//        SmartDashboard.putBoolean("MetricTracker/Drive", true);
        super.initialize();
        // create the _angleController here, just like in AutoDriveToTarget
        _driveState = DriveState.normal;
        _targetSighted = false;
        _angleController = new PIDController(Constants.Auto.Drive.AnglePID.kP,Constants.Auto.Drive.AnglePID.kI,Constants.Auto.Drive.AnglePID.kD, _imu, new AngleListener(), 0.1);
        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _angleController.setOutputRange(-Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE, Constants.Auto.Drive.AnglePID.MAX_DIFFERENCE);
        _angleController.setAbsoluteTolerance(Constants.Auto.Drive.AnglePID.TOLERANCE);
        _angleController.setContinuous();

        _mediumZone = Robot.pickConstant(Constants.DriveTrain.MEDIUM_ZONE_COMP, Constants.DriveTrain.MEDIUM_ZONE_PROTO);
        _slowZone = Robot.pickConstant(Constants.DriveTrain.SLOW_ZONE_COMP, Constants.DriveTrain.SLOW_ZONE_PROTO);

        _mediumSpeed = Robot.pickConstant(Constants.DriveTrain.MEDIUM_SPEED_COMP, Constants.DriveTrain.MEDIUM_SPEED_PROTO);
        _slowSpeed = Robot.pickConstant(Constants.DriveTrain.SLOW_SPEED_COMP, Constants.DriveTrain.SLOW_SPEED_PROTO);

    }

    @Override
    protected void execute() {
        super.execute();
        // Get the base speed from the throttle
        double stickSpeed = _oi.getDriveSpeed();

        // Get the rotation from the tiller
        double wheelRotation = _oi.getDriveRotation();
        _targetSighted = _limelight.isTargetSighted();
        if (!_oi.isAutoTargetPressed() || !_elevator.isLimelightClear()) {
            _stickyLimit = 1.0;
            _lockout = false;
            if (_driveState!=DriveState.normal) {
                // Stop tracking
                _limelight.disableLEDs();
                _driveState = DriveState.normal;
            }
            if (wheelRotation==0 && stickSpeed != 0) {
                // Driving straight...lock in the yaw
                if (!_angleController.isEnabled()) {
                    _anglePIDOut = 0;
                    double yaw = _imu.getYaw();
                    metric("PID/SetPoint", yaw);
                    _angleController.setSetpoint(yaw);
                    _angleController.setPID(Constants.Auto.Drive.AnglePID.kP, Constants.Auto.Drive.AnglePID.kI, Constants.Auto.Drive.AnglePID.kD);
                    _angleController.enable();
                    metric("PID/Enabled", true);
                }
            } else if (_angleController.isEnabled()) {
                _angleController.disable();
                _anglePIDOut = 0;
                metric("PID/Enabled", false);
            }
        } else {
            switch (_driveState) {
                case normal:
                    // Start seeking
                    if (_hatchIntake.isDown()) {
                        if (_cargoIntake.isIntaking()) {
                            error("Cargo intaking");
                            _limelight.setPipeline(Limelight.Pipeline.CargoTrackingLargest);
                            metric("Pipeline", Limelight.Pipeline.CargoTrackingLargest.name());
                            _limelight.enableLEDs();
                            _driveState = DriveState.seekingcargo;
                        } else {
                            _limelight.setPipeline(Limelight.Pipeline.TapeTrackingHighest);
                            metric("Pipeline", Limelight.Pipeline.TapeTrackingHighest.name());
                            _limelight.enableLEDs();
                            _driveState = DriveState.seekingport;
                            _seekMax = System.currentTimeMillis() + Constants.DriveTrain.SEEK_TIME;
                        }
                    } else {
                        _limelight.setPipeline(Limelight.Pipeline.TapeTrackingLargest);
                        metric("Pipeline", Limelight.Pipeline.TapeTrackingLargest.name());
                        _limelight.enableLEDs();
                        _driveState = DriveState.seeking;
                        _seekMax = System.currentTimeMillis() + Constants.DriveTrain.SEEK_TIME;
                    }
                    break;
                case seeking:
                    if (_targetSighted) {
                        _turnSpeed = getTurnSpeed();
                        _lockEnd = System.currentTimeMillis() + Constants.DriveTrain.LOCK_TIME;
                        _driveState = DriveState.locking;
                    }
                    break;
                case seekingport:
                    if (_targetSighted) {
                        _turnSpeed = getTurnSpeed();
                        _lockEnd = System.currentTimeMillis() + Constants.DriveTrain.LOCK_TIME;
                        _driveState = DriveState.lockingport;
                    }
                    break;
                case seekingcargo:
                    if (_limelight.isTargetSighted()) {
                        _turnSpeed = getTurnSpeed();
                        if (_limelight.isTargetCentered()) {
                            _limelight.setPipeline(Limelight.Pipeline.CargoTrackingClosest);
                            metric("Pipeline", Limelight.Pipeline.CargoTrackingClosest.name());
                            _driveState = DriveState.trackingcargo;
                        }
                    }
                    break;
                case locking:
                    if (System.currentTimeMillis() > _lockEnd || (_limelight.isTargetSighted() && _limelight.isTargetCentered())) {
                        // Note that we could also wait until the target is centered to lock...which might make more sense.
                        // Just add  && _limelight.isTargetCentered() to the condition above
                        _limelight.setPipeline(Limelight.Pipeline.TapeTrackingClosest);
                        metric("Pipeline", Limelight.Pipeline.TapeTrackingClosest.name());
                        _driveState = DriveState.tracking;
                    }
                    _turnSpeed = getTurnSpeed();
                    break;
                case lockingport:
                    if (System.currentTimeMillis() > _lockEnd || (_limelight.isTargetSighted() && _limelight.isTargetCentered())) {
                        // Note that we could also wait until the target is centered to lock...which might make more sense.
                        // Just add  && _limelight.isTargetCentered() to the condition above
                        //_limelight.setPipeline(Limelight.Pipeline.TapeTrackingClosest);
                        //metric("Pipeline", Limelight.Pipeline.TapeTrackingClosest.name());
                        _driveState = DriveState.tracking;
                    }
                    _turnSpeed = getTurnSpeed();
                    break;
                case tracking:
                    _turnSpeed = getTurnSpeed();
                    break;
                case trackingcargo:
                    _turnSpeed = getTurnSpeed();
                    break;
            }
        }

        metric("State", _driveState.name());
        stickSpeed = limitSpeed(stickSpeed);
        if(!_oi.isOverridePressed() && _hatchIntake.isShockTriggered()) {
            _driveTrain.cheesyDrive(Math.min(stickSpeed, 0), 0, false, false);
        } else if (_driveState == DriveState.normal) {
            if (wheelRotation==0 && _angleController.isEnabled()) {
                metric("PID/AngleOut", _anglePIDOut);
                metric("PID/Yaw", _imu.getYaw());
                _driveTrain.cheesyDrive(stickSpeed, stickSpeed==0 ?  0 :_anglePIDOut, false, true);
            } else {
                _driveTrain.cheesyDrive(stickSpeed, wheelRotation, _oi.isCreepPressed(), false);
            }
        } else {
            _driveTrain.cheesyDrive(stickSpeed, _turnSpeed, false, true);
        }
        metric("StickSpeed", stickSpeed);
        metric("StickRotation", wheelRotation);
        metric("LeftPower", _driveTrain.getLeftPower());
        metric("RightPower", _driveTrain.getRightPower());
        // metric("LeftMasterAmps", _driveTrain.getLeftMasterCurrent());
        // metric("LeftFollowerAmps",_driveTrain.getLeftFollowerCurrent());
        // metric("RightMasterAmps",_driveTrain.getRightMasterCurrent());
       //  metric("RightFollowerAmps",_driveTrain.getRightFollowerCurrent());
        metric("TurnSpeed", _turnSpeed);
    }
    protected double getTurnSpeed() {
        metric("Lockout", _lockout);
        if (_lockout || !_limelight.isTargetSighted()) { return 0; }
        double distance = _limelight.getTargetDistance();

        _seekMax = System.currentTimeMillis() + Constants.DriveTrain.DROPOUT_TIME;

        if (distance > 0 && distance < Constants.Auto.Drive.MIN_TRACK_DISTANCE) {
            // We're too close to trust the target!
            error("Target too close at " + distance + ", count=" + garbageCount);
            garbageCount++;
            if (garbageCount > Constants.Auto.Drive.MAX_GARBAGE) {
                _lockout = true;
                error("Garbagecount=" + garbageCount + " setting lockout.");
            }
            return 0;
        }
        garbageCount = 0;
        double limeLightAngle = _limelight.getHorizontalAngle();
        double yaw = _imu.getYaw();

        // Find the pose of the robot _when the picture was taken_
        long timeKey = System.currentTimeMillis() - (long)_limelight.getLatency();
        BasicPose pose = (BasicPose)_poseTracker.get(timeKey);

        // Get the angle from the pose if one was found--otherwise use yaw
        double poseAngle = pose == null ? yaw : pose.getAngle();

        // Now adjust the limelight angle based on the change in yaw from when the picture was taken to now
        double offsetCompensation = yaw - poseAngle;
        double targetAngle = limeLightAngle - offsetCompensation;

        metric("Pose", pose==null?0:pose.getMillis());
        metric("Yaw", yaw);
        metric("PoseAngle", poseAngle);
        metric("LimelightAngle", limeLightAngle);
        metric("TargetAngle", targetAngle);

        return targetAngle * STEER_K;
    }

    private double limitSpeed(double speed) {
        double limit = 1;
        if (_driveState!=DriveState.normal) {
            if(_limelight.isTargetSighted()) {
                _seekMax = System.currentTimeMillis() + Constants.DriveTrain.DROPOUT_TIME;
                double distance = _limelight.getTargetDistance();
                metric("TargetDistance", distance);
                if (distance  > 0) {
                    if (distance < _mediumZone) {
                        limit = _mediumSpeed;
                        _stickyLimit = limit;
                    }
                    if (distance < _slowZone) {
                        limit = _slowSpeed;
                        _stickyLimit = limit;
                    }
                }
            } else if (System.currentTimeMillis() > _seekMax){
                metric("TargetDistance", -999);
                // We've been seeking for more than the max allowed...slow the robot down!
                _oi.pulseDriver(1);
            }
        }
        limit = Math.min(limit, _stickyLimit);
        if (_elevator.isAboveMiddle()) {
            error("Limiting speed due to elevator!");
            limit = Math.min(0.5, limit);
        }
        double limited = Helpers.limit(speed, -limit, limit);
        metric("limit", limit);
        metric("limited", limited);
        return limited;
    }

    @Override
    protected boolean isFinished() {
        return false;
    }

    private class AngleListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _anglePIDOut = output;
            }
        }

    }

    public enum DriveState {
        normal(0),
        seeking(1),
        seekingport(2),
        locking(3),
        lockingport(4),
        tracking(5),
        seekingcargo(6),
        trackingcargo(7),
        lost(8);

        private int _value;

        DriveState(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }
}
