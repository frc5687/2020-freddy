package org.frc5687.freddy2020.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.*;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.OI;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.RobotMap;
import org.frc5687.freddy2020.robot.commands.Drive;
import org.frc5687.freddy2020.robot.utils.IRDistanceSensor;
import org.frc5687.freddy2020.robot.utils.Limelight;
import org.frc5687.freddy2020.robot.utils.PDP;
import org.frc5687.freddy2020.robot.utils.PoseTracker;

import static org.frc5687.freddy2020.robot.Constants.DriveTrain.CREEP_FACTOR;
import static org.frc5687.freddy2020.robot.utils.Helpers.applySensitivityFactor;
import static org.frc5687.freddy2020.robot.utils.Helpers.limit;

public class DriveTrain extends OutliersSubsystem implements PIDSource {
    private CANSparkMax _leftMaster;
    private CANSparkMax _rightMaster;

    private CANSparkMax _leftFollower;
    private CANSparkMax _rightFollower;;

    private Encoder _leftMagEncoder;
    private Encoder _rightMagEncoder;

    private OI _oi;
    private AHRS _imu;
    private Limelight _limelight;
    private Robot _robot;

    private double _oldLeftSpeed;
    private double _oldLeftSpeedF;
    private double _oldRightSpeed;
    private double _oldRightSpeedF;
    private boolean _isPaused = false;

    private Shifter _shifter;
    private PDP _pdp;

    public DriveTrain(Robot robot) {
        info("Constructing DriveTrain class.");
        _oi = robot.getOI();
        _imu = robot.getIMU();
        _limelight = robot.getLimelight();
        _pdp = robot.getPDP();
        _robot = robot;

        _shifter = robot.getShifter();

        try {
            debug("Allocating motor controllers");
            _leftMaster = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_LEFT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightMaster = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_RIGHT_MASTER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _leftFollower = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_LEFT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightFollower = new CANSparkMax(RobotMap.CAN.SPARKMAX.DRIVE_RIGHT_FOLLOWER, CANSparkMaxLowLevel.MotorType.kBrushless);

            _leftMaster.restoreFactoryDefaults();
            _rightMaster.restoreFactoryDefaults();
            _leftFollower.restoreFactoryDefaults();
            _rightFollower.restoreFactoryDefaults();


            _leftMaster.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _rightMaster.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _leftFollower.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _rightFollower.setOpenLoopRampRate(Constants.DriveTrain.RAMP_RATE);

            _leftMaster.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _rightMaster.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _leftFollower.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);
            _rightFollower.setClosedLoopRampRate(Constants.DriveTrain.RAMP_RATE);

            _leftMaster.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);
            _rightMaster.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);
            _leftFollower.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);
            _rightFollower.setSmartCurrentLimit(Constants.DriveTrain.STALL_CURRENT_LIMIT, Constants.DriveTrain.FREE_CURRENT_LIMIT);

            _leftMaster.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);
            _rightMaster.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);
            _leftFollower.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);
            _rightFollower.setSecondaryCurrentLimit(Constants.DriveTrain.SECONDARY_LIMIT);

            _leftMaster.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
            _leftFollower.setInverted(Constants.DriveTrain.LEFT_MOTORS_INVERTED);
            _rightMaster.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);
            _rightFollower.setInverted(Constants.DriveTrain.RIGHT_MOTORS_INVERTED);

            enableBrakeMode();

        } catch (Exception e) {
            error("Exception allocating drive motor controllers: " + e.getMessage());
        }

        debug("Configuring mag encoders");
        _leftMagEncoder = new Encoder(RobotMap.DIO.DRIVE_LEFT_A, RobotMap.DIO.DRIVE_LEFT_B);
        _rightMagEncoder = new Encoder(RobotMap.DIO.DRIVE_RIGHT_A, RobotMap.DIO.DRIVE_RIGHT_B);
        _leftMagEncoder.setDistancePerPulse(Constants.DriveTrain.LEFT_DISTANCE_PER_PULSE);
        _rightMagEncoder.setDistancePerPulse(Constants.DriveTrain.RIGHT_DISTANCE_PER_PULSE);
        resetDriveEncoders();


//        logMetrics("Power/Left", "Power/Right",
//                "PDPCurrent/LeftMaster", "PDPCurrent/RightMaster", "PDPCurrent/LeftFollower", "PDPCurrent/RightFollower",
//                "Applied/LeftMaster", "Applied/RightMaster", "Applied/LeftFollower","Applied/RightFollower",
//                "OutputCurrent/LeftMaster", "OutputCurrent/RightMaster", "OutputCurrent/LeftFollower", "OutputCurrent/RightFollower");
    }

    public void enableBrakeMode() {
        _leftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _leftFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _rightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _rightFollower.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void disableBrakeMode() {
        _leftMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _leftFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _rightMaster.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _rightFollower.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    private boolean assertMotorControllers() {
        if (_leftMaster==null) { return false; }
        if (_rightMaster==null) { return false; }
        return true;
    }

    @Override
    public void updateDashboard() {
//        metric("Neo/Ticks/Left", getLeftTicks());
//        metric("Neo/Ticks/Right", getRightTicks());
//        metric("Neo/Distance/Left", getLeftDistance());
//        metric("Neo/Distance/Right", getRightDistance());
//        metric("Neo/Distance/Total", getDistance());
        metric("Ticks/Left", _leftMagEncoder.get());
        metric("Ticks/Right", _rightMagEncoder.get());
        metric("MagDistance/Left", _leftMagEncoder.getDistance());
        metric("MagDistance/Right", _rightMagEncoder.getDistance());
        metric("Distance/Left", getLeftDistance());
        metric("Distance/Right", getRightDistance());

        //        metric("Faults/LeftMaster", _leftMaster.getFaults());
//        metric("Faults/RightMaster", _rightMaster.getFaults());
//        metric("Faults/LeftFollower", _leftFollower.getFaults());
//        metric("Faults/RightFollower", _rightFollower.getFaults());
//        metric("Applied/LeftMaster", _leftMaster.getAppliedOutput());
//        metric("Applied/RightMaster", _rightMaster.getAppliedOutput());
//        metric("Applied/LeftFollower", _leftFollower.getAppliedOutput());
//        metric("Applied/RightFollower", _rightFollower.getAppliedOutput());
//        metric("OutputCurrent/LeftMaster", _leftMaster.getOutputCurrent());
//        metric("OutputCurrent/RightMaster", _rightMaster.getOutputCurrent());
//        metric("OutputCurrent/LeftFollower", _leftFollower.getOutputCurrent());
//        metric("OutputCurrent/RightFollower", _rightFollower.getOutputCurrent());
//        metric("PDPCurrent/LeftMaster", getLeftMasterCurrent());
//        metric("PDPCurrent/RightMaster", getRightMasterCurrent());
//        metric("PDPCurrent/LeftFollower", getLeftFollowerCurrent());
//        metric("PDPCurrent/RightFollower", getRightFollowerCurrent());


    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new Drive(this, _imu, _oi, _limelight, _robot.getElevator(), _robot.getCargoIntake(),_robot.getHatchIntake(), _robot.getPoseTracker()));
    }

    public void cheesyDrive(double speed, double rotation, boolean creep, boolean override) {
        if (!assertMotorControllers()) { return; }
        metric("Speed", speed);
        metric("Rotation", rotation);

        speed = limit(speed, 1);
        //Shifter.Gear gear = _robot.getShifter().getGear();

        rotation = limit(rotation, 1);

        double leftMotorOutput;
        double rightMotorOutput;

        double maxInput = Math.copySign(Math.max(Math.abs(speed), Math.abs(rotation)), speed);

        if (speed < Constants.DriveTrain.DEADBAND && speed > -Constants.DriveTrain.DEADBAND) {
            //metric("Rot/Raw", rotation);
            if (!override) {
                rotation = applySensitivityFactor(rotation, _shifter.getGear() == Shifter.Gear.HIGH ? Constants.DriveTrain.ROTATION_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.ROTATION_SENSITIVITY_LOW_GEAR);
            }
            if (creep) {
                //metric("Rot/Creep", creep);
                rotation = rotation * CREEP_FACTOR;
            } else {
                rotation = rotation * 1;
            }

            //metric("Rot/Transformed", rotation);
            leftMotorOutput = rotation;
            rightMotorOutput = -rotation;
        } else {
            // Square the inputs (while preserving the sign) to increase fine control
            // while permitting full power.
            metric("Str/Raw", speed);
            speed = Math.copySign(applySensitivityFactor(speed, Constants.DriveTrain.SPEED_SENSITIVITY), speed);
            metric("Str/Trans", speed);
            if (!override) {
                rotation = applySensitivityFactor(rotation, _shifter.getGear() == Shifter.Gear.HIGH ? Constants.DriveTrain.TURNING_SENSITIVITY_HIGH_GEAR : Constants.DriveTrain.TURNING_SENSITIVITY_LOW_GEAR);
            }
            double delta = override ? rotation : rotation;//* Math.abs(speed)


            if (override) {
                // speed = Math.copySign(limit(Math.abs(speed), 1-Math.abs(delta)), speed);

                if (speed + Math.abs(delta) > 1) {
                    speed = 1 - Math.abs(delta);
                }

                if (speed - Math.abs(delta) < -1) {
                    speed = -1 + Math.abs(delta);
                }
            }

            leftMotorOutput = speed + delta;
            rightMotorOutput = speed - delta;
        }
        setPower(limit(leftMotorOutput), limit(rightMotorOutput), true);
    }

    public float getYaw() {
        return _imu.getYaw();
    }


    public void setPower(double leftSpeed, double rightSpeed, boolean override) {
//         if (!assertMotorControllers()) { return; }
        if (_isPaused == true) {
            leftSpeed = 0;
            rightSpeed = 0;
        }
        try {
            _leftMaster.set(leftSpeed);
            _leftFollower.set(leftSpeed);
            _rightMaster.set(rightSpeed);
            _rightFollower.set(rightSpeed);
        } catch (Exception e) {
            error("DriveTrain.setPower exception: " + e.toString());
        }
        metric("Power/Right", rightSpeed);
        metric("Power/Left", leftSpeed);
    }
    public void pauseMotors() {
        _oldLeftSpeed = _leftMaster.get();
        _oldLeftSpeedF = _leftFollower.get();
        _oldRightSpeed = _rightMaster.get();
        _oldRightSpeedF = _rightFollower.get();
        _leftMaster.set(0);
        _leftFollower.set(0);
        _rightMaster.set(0);
        _rightFollower.set(0);
        _isPaused = true;
    }

    public void resumeMotors() {
        _leftMaster.set(_oldLeftSpeed);
        _leftFollower.set(_oldLeftSpeedF);
        _rightMaster.set(_oldRightSpeed);
        _rightFollower.set(_oldRightSpeedF);
        _isPaused = false;
    }

    public double getLeftDistance() {
        return getLeftTicks() * Constants.DriveTrain.LEFT_DISTANCE_PER_PULSE;
    }

    public double getRightDistance() {
        return getRightTicks() * Constants.DriveTrain.RIGHT_DISTANCE_PER_PULSE;
    }

    public double getLeftTicks() {
        return _leftMagEncoder.get();
    }

    public double getRightTicks() {
        return _rightMagEncoder.get();
    }

    public double getDistance() {
        if (Math.abs(getRightTicks())<10) {
            return getLeftDistance();
        }
        if (Math.abs(getLeftTicks())<10) {
            return getRightDistance();
        }
        return (getLeftDistance() + getRightDistance()) / 2;
    }

    public void resetDriveEncoders() {
        _leftMagEncoder.reset();
        _rightMagEncoder.reset();
    }

    @Override
    public double pidGet() {
        return getDistance();
    }

    @Override
    public PIDSourceType getPIDSourceType() {
        return PIDSourceType.kDisplacement;
    }

    @Override
    public void setPIDSourceType(PIDSourceType pidSource) {
    }


    public double getLeftPower() {
        return _leftMaster.get();
    }

    public double getRightPower() {
        return _rightMaster.get();
    }


    public double getLeftMasterCurrent() {
        return _pdp.getCurrent(RobotMap.PDP.DRIVE_LEFT_MASTER);
    }
    public double getLeftFollowerCurrent() {
        return _pdp.getCurrent(RobotMap.PDP.DRIVE_LEFT_FOLLOWER);
    }
    public double getRightMasterCurrent() {
        return _pdp.getCurrent(RobotMap.PDP.DRIVE_RIGHT_MASTER);
    }
    public double getRightFollowerCurrent() {
        return _pdp.getCurrent(RobotMap.PDP.DRIVE_RIGHT_FOLLOWER);
    }

}
