package org.frc5687.freddy2020.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.RobotMap;
import org.frc5687.freddy2020.robot.commands.DriveArm;
import org.frc5687.freddy2020.robot.utils.HallEffect;
import org.frc5687.freddy2020.robot.utils.PDP;

import static org.frc5687.freddy2020.robot.Constants.Arm.*;
import static org.frc5687.freddy2020.robot.utils.Helpers.*;

public class Arm extends OutliersSubsystem  {

    private CANSparkMax _leftSpark;
    private CANSparkMax _rightSpark;

    private CANEncoder _leftEncoder;
    private CANEncoder _rightEncoder;

    private HallEffect _rightStowedhall;
    private HallEffect _leftStowedhall;
    private PDP _pdp;

    private double _leftOffset = 0;
    private double _rightOffset = 0;

    private double _lastLeftSpeed;
    private double _lastRightSpeed;
    private Robot _robot;

    private int _pdpLeft;
    private int _pdpRight;
    private boolean _leftEncoderZeroed = false;
    private boolean _rightEncoderZeroed = false;

    public Arm(Robot robot) {
        _robot = robot;

        try {
            _leftSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.LEFT_ARM, CANSparkMaxLowLevel.MotorType.kBrushless);
            _rightSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.RIGHT_ARM, CANSparkMaxLowLevel.MotorType.kBrushless);


            _leftSpark.setInverted(Constants.Arm.LEFT_MOTOR_INVERTED);
            _rightSpark.setInverted(Constants.Arm.RIGHT_MOTOR_INVERTED);

            _leftSpark.setSmartCurrentLimit(Constants.Arm.CLIMB_STALL_LIMIT, Constants.Arm.CLIMB_FREE_LIMIT);
            _rightSpark.setSmartCurrentLimit(Constants.Arm.CLIMB_STALL_LIMIT, Constants.Arm.CLIMB_FREE_LIMIT);

            _leftEncoder = _leftSpark.getEncoder();
            _rightEncoder  = _rightSpark.getEncoder();

            _pdpLeft = _robot.getIdentityMode() == Robot.IdentityMode.practice ? RobotMap.PDP.LEFT_ARM_PRACTICE : RobotMap.PDP.LEFT_ARM_COMPO;
            _pdpRight = _robot.getIdentityMode() == Robot.IdentityMode.practice ? RobotMap.PDP.RIGHT_ARM_PRACTICE : RobotMap.PDP.RIGHT_ARM_COMP;

        } catch (Exception e) {
            error("Unable to allocate arm controller: " + e.getMessage());
        }
        _pdp = robot.getPDP();
        _rightStowedhall = new HallEffect(RobotMap.DIO.ARM_RIGHT_STOWED_HALL);
        _leftStowedhall = new HallEffect(RobotMap.DIO.ARM_LEFT_STOWED_HALL);
    }

    public void enableBrakeMode() {
        if (_leftSpark ==null) { return; }
        _leftSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
        _rightSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableCoastMode() {
        if (_leftSpark ==null) { return; }
        _leftSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
        _rightSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public void setSpeed(double speed) {
        if (_leftSpark == null || _rightSpark==null) { return; }
        setLeftSpeed(speed);
        setRightSpeed(speed);
    }


    public void setLeftSpeed(double speed) {
        if (_leftSpark == null) { return; }

        if(_lastLeftSpeed >= 0 && speed < 0){
            _leftSpark.setSmartCurrentLimit(Constants.Arm.STOW_STALL_LIMIT);
        } else if (_lastLeftSpeed < 0 && speed >= 0){
            _leftSpark.setSmartCurrentLimit(Constants.Arm.CLIMB_STALL_LIMIT);
        }
        _lastLeftSpeed = speed;

        speed = limit(speed,
                isLeftStowed() ? 0 : STOW_SPEED,
                isLeftLow() ? HOLD_SPEED : MAX_DRIVE_SPEED);
        metric("LeftSpeed", speed);
        if(isLeftStowed()) { resetLeftEncoder(); }
        _leftSpark.set(speed);
    }

    public void setRightSpeed(double speed) {
        if (_rightSpark == null) { return; }

        if(_lastRightSpeed >= 0 && speed < 0){
            _rightSpark.setSmartCurrentLimit(Constants.Arm.STOW_STALL_LIMIT);
        } else if (_lastRightSpeed < 0 && speed >= 0){
            _rightSpark.setSmartCurrentLimit(Constants.Arm.CLIMB_STALL_LIMIT);
        }
        _lastRightSpeed = speed;


        speed = limit(speed,
                isRightStowed() ? 0 : STOW_SPEED,
                isRightLow() ? HOLD_SPEED : MAX_DRIVE_SPEED);
        metric("RightSpeed", speed);
        if(isRightStowed()) { resetRightEncoder(); }
        _rightSpark.set(speed);
    }


    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveArm(_robot, this));
    }

    @Override
    public void updateDashboard() {
        metric("StowedRightHall", _rightStowedhall.get());
        metric("StowedLeftHall", _leftStowedhall.get());
        metric("LeftEncoder", getLeftPosition());
        metric("RightEncoder", getRightPosition());
        metric("Position", getPosition());
        metric("Angle", getAngle());
        metric("EncodersZeroed", encodersZeroed());
    }

    public boolean isStowed() {
        return isLeftStowed() || isRightStowed();

    }

    public boolean isRightStowed() {
        return _rightStowedhall.get()
                ||_lastRightSpeed < 0 && _pdp.getCurrent(_pdpRight) > Constants.Arm.STOW_STALL_THRESHOLD;
    }

    public boolean isLeftStowed() {
        return _leftStowedhall.get()
                || _lastLeftSpeed < 0 && _pdp.getCurrent(_pdpLeft) > Constants.Arm.STOW_STALL_THRESHOLD;
    }

    public boolean isRightLow() {
        return false;
    }

    public boolean isLeftLow() {
        return false;
    }


    public boolean isLow() {
        return isLeftLow() || isRightLow();
    }




    public double getPosition() {
        return (getLeftPosition() + getRightPosition())/2;
    }

    public double getLeftPosition() {
        return _leftEncoder.getPosition() - _leftOffset;
    }

    public double getRightPosition() {
        return _rightEncoder.getPosition() - _rightOffset;
    }

    public void resetEncoders() {
        resetLeftEncoder();
        resetRightEncoder();
    }

    public void resetLeftEncoder() {
        _leftEncoderZeroed = true;
        _leftOffset = _leftEncoder.getPosition();
    }

    public void resetRightEncoder() {
        _rightEncoderZeroed = true;
        _rightOffset = _rightEncoder.getPosition();
    }

    public double getAngle() {
        return Constants.Arm.STOWED_ANGLE + (getPosition() * Constants.Arm.DEGREES_PER_TICK);
    }

    public boolean encodersZeroed() {
        return _leftEncoderZeroed && _rightEncoderZeroed;
    }
}


