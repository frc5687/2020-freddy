package org.frc5687.freddy2020.robot.commands;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.subsystems.DriveTrain;
import org.frc5687.freddy2020.robot.subsystems.HatchIntake;
import org.frc5687.freddy2020.robot.utils.Limelight;
import static org.frc5687.freddy2020.robot.Constants.Auto.DriveToTarget.*;

public class AutoDriveToTarget extends OutliersCommand {
    private DriveTrain _driveTrain;
    private AHRS _imu;
    private Limelight _limelight;
    private OI _oi;

    private PIDController _angleController;
    private PIDController _distanceController;


    private double _angleTarget;
    private double _desiredOffset;
    private double _distanceTolerance;

    private double _speed;
    private double _distanceToTarget;

    private double _anglePIDOut;
    private double _distancePIDOut;

    private long _startTimeMillis;
    private boolean _aborted = false;
    private boolean _ignoreDistance = false;

    private String _stage = "";

    public AutoDriveToTarget (Robot robot, double speed, double distance, double tolerance, String stage) {
        _driveTrain = robot.getDriveTrain();
        _imu = robot.getIMU();
        _limelight = robot.getLimelight();
        _oi = robot.getOI();

        requires(_driveTrain);
        _desiredOffset = distance;
        _speed = speed;
        _distanceTolerance = tolerance;
        _stage = stage;
    }

    @Override
    protected void initialize() {
        _aborted = false;
        _ignoreDistance = false;
        _driveTrain.resetDriveEncoders();
        _startTimeMillis = System.currentTimeMillis();
        _limelight.enableLEDs();
        error("Running AutoDriveToTarget to " + _desiredOffset + " inches at " + _speed);

//        boolean irMode = false;

//        _distanceToTarget = _driveTrain.getFrontDistance();
//        if (_distanceToTarget > Constants.Auto.IR_THRESHOLD) {
//            irMode = false;
//            if (_limelight.isTargetSighted()) {
//                _distanceToTarget = _limelight.getTargetDistance();
//            } else {
//                _aborted = true;
//            }
//        }
//        metric("distance/IRMode", irMode);
            // 1: Read current target _angleTarget from limelight
            // 2: Read current yaw from navX
            // 3: Set _angleController._angleTarget to sum

            // If we can't see the target, don't use limelight's angle!
        _distanceController = new PIDController(kPDistance, kIDistance, kDDistance, _driveTrain, new DistanceListener(), 0.1);
        _distanceController.setOutputRange(-_speed, _speed);
        _distanceController.setAbsoluteTolerance(_distanceTolerance);
        _distanceController.setContinuous(false);

        _angleController = new PIDController(kPAngle, kIAngle, kDAngle, _imu, new AngleListener(), 0.1);

        _angleController.setInputRange(Constants.Auto.MIN_IMU_ANGLE, Constants.Auto.MAX_IMU_ANGLE);
        _angleController.setOutputRange(-TURN_SPEED, TURN_SPEED);
        _angleController.setAbsoluteTolerance(ANGLE_TOLERANCE);
        _angleController.setContinuous();
        metric("angle/startyaw", _imu.getYaw());

    }


    @Override
    protected void execute() {
        _driveTrain.enableBrakeMode();

        if (_limelight.isTargetSighted()) {
            _distanceToTarget = _limelight.getTargetDistance();
            double destination = _driveTrain.getDistance() + _distanceToTarget - _desiredOffset;
            double oldDestination = _distanceController.isEnabled() ? _distanceController.getSetpoint() : _driveTrain.getDistance();
            metric("distance/target", destination);

            if (destination < 24) {
                _ignoreDistance = true;
            }

            if (!_ignoreDistance && (!_distanceController.isEnabled() || Math.abs(oldDestination - _driveTrain.getDistance()) > _distanceTolerance)) {
                _distanceController.setSetpoint(destination);
                metric("distance/setpoint", destination);
                _distanceController.enable();
            }

        }


//        if (_speed > 0.3 && Math.abs(oldDestination - _driveTrain.getDistance()) <= 36) {
//            DriverStation.reportError("CAPPING", false);
//            _distanceController.setOutputRange(-0.3, 0.3);
//            _distanceController.enable();
//        }

        if (_limelight.isTargetSighted()) {
            double limeLightAngle = _limelight.getHorizontalAngle();
            double yawAngle = _imu.getYaw();
            _angleTarget = limeLightAngle + yawAngle;


            if (!_angleController.isEnabled()) {
                _angleController.setSetpoint(_angleTarget);
                _angleController.enable();
                metric("angle/startoffset", limeLightAngle);
                metric("angle/target", _angleTarget);
            }

//            if (Math.abs(_angleTarget - _angleController.getSetpoint()) > Constants.Auto.DriveToTarget.ANGLE_TOLERANCE) {
//                _angleController.setSetpoint(_angleTarget);
//                metric("angle/setpoint", _angleTarget);
//            }
        }
//



        metric("angle/yaw", _imu.getYaw());
        metric("distance/currentTargetDistance", _distanceToTarget);
        metric("angle/PIDOut", _anglePIDOut);
        metric("distance/PIDOut", _distancePIDOut);
        metric("distance/current", _driveTrain.getDistance());
//        metric("distance/IRMode", irMode);
        _driveTrain.setPower(_distancePIDOut + _anglePIDOut , _distancePIDOut - _anglePIDOut, true); // positive output is clockwise

    }
    @Override
    protected boolean isFinished() {
        if (!_limelight.isTargetSighted()) { return true; }
        if (_distanceToTarget < 18) {
            if (_limelight.getTargetArea() > 6.5 && _limelight.getTargetArea() < 7.5) {
                return true;
            }
        }
        if (_aborted) { return true; }

        if (_distanceController.onTarget()) {
            return true;
        }

        if (Math.abs(_distanceToTarget - _desiredOffset) <= _distanceTolerance) { return true; }

        return false;
    }

    @Override
    protected void end() {
        //   _limelight.disableLEDs();
        _driveTrain.enableBrakeMode();
        _driveTrain.setPower(0,0, true);
        error("AutoDriveToTarget finished: angle=" + _imu.getYaw() + ", distance=" + _distanceToTarget + ", time=" + (System.currentTimeMillis() - _startTimeMillis));
        _distanceController.disable();
        _angleController.disable();
    }
    private class AngleListener implements PIDOutput {


        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _anglePIDOut = output;
            }
        }

    }
    private class DistanceListener implements PIDOutput {

        @Override
        public void pidWrite(double output) {
            synchronized (this) {
                _distancePIDOut = output;
            }
        }
    }

}

