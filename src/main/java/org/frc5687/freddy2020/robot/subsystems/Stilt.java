package org.frc5687.freddy2020.robot.subsystems;

import com.revrobotics.CANEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.RobotMap;
import org.frc5687.freddy2020.robot.commands.stilt.DriveStilt;
import org.frc5687.freddy2020.robot.utils.HallEffect;
import org.frc5687.freddy2020.robot.utils.IRDistanceSensor;
import static org.frc5687.freddy2020.robot.Constants.Stilt.*;
import static org.frc5687.freddy2020.robot.utils.Helpers.*;

public class Stilt extends OutliersSubsystem {

    private CANSparkMax _lifterSpark;
    private CANEncoder _neoStiltEncoder;
    private PWMVictorSPX _wheelieVictor;

    private Robot _robot;

    private HallEffect _retractedHall;
    private HallEffect _middleHall;
    private HallEffect _extendedHall;

    private double _offset = 0;

    private IRDistanceSensor _downIR = new IRDistanceSensor(RobotMap.Analog.DOWN_IR, IRDistanceSensor.Type.SHORT);

    public Stilt(Robot robot) {
        _robot = robot;

        try {
            _lifterSpark = new CANSparkMax(RobotMap.CAN.SPARKMAX.STILT, CANSparkMaxLowLevel.MotorType.kBrushless);
            _lifterSpark.setInverted(MOTOR_INVERTED);
            _neoStiltEncoder = _lifterSpark.getEncoder();
            enableBrakeMode();
        }catch (Exception e) {
            error("Unable to allocate stilt controller: " + e.getMessage());
        }

        _retractedHall = new HallEffect(RobotMap.DIO.STILT_RETRACTED_HALL);
        _middleHall = new HallEffect(RobotMap.DIO.STILT_MIDDLE);
        _extendedHall = new HallEffect(RobotMap.DIO.STILT_EXTENDED_HALL);

        _wheelieVictor = new PWMVictorSPX(RobotMap.PWM.Wheelie);
        _wheelieVictor.setInverted(false);
    }

    public void setLifterSpeed(double desiredSpeed) {
        double speed = desiredSpeed; // limit(desiredSpeed, isExtended() ? 0 : -MAX_DOWN_SPEED , isRetracted() ? 0 : MAX_UP_SPEED);
        metric("Lifter/rawSpeed", desiredSpeed);
        metric("Lifter/speed", speed);
        _lifterSpark.set(speed);
    }

    public void setWheelieSpeed(double desiredSpeed) {
        double speed = limit(desiredSpeed, -1, 1);
        metric("Wheelie/rawSpeed", desiredSpeed);
        metric("Wheelie/speed", speed);
        _wheelieVictor.set(speed);
    }

    public void enableBrakeMode() {
        if (_lifterSpark ==null) { return; }
        _lifterSpark.setIdleMode(CANSparkMax.IdleMode.kBrake);
    }

    public void enableCoastMode() {
        if (_lifterSpark ==null) { return; }
        _lifterSpark.setIdleMode(CANSparkMax.IdleMode.kCoast);
    }

    public double getRawNeoEncoder() {
        if (_lifterSpark ==null) { return 0; }
        return _neoStiltEncoder.getPosition();
    }

    public double getPosition() {
        return getRawNeoEncoder() - _offset;
    }

    public boolean isRetracted() {
        return _retractedHall.get();// || (Math.abs(getPosition()-TOP_POSITION)<TOLERANCE);
    }

    public boolean isExtended() {
        return _extendedHall.get();// || (Math.abs(getPosition()-BOTTOM_POSITION)<TOLERANCE);
    }

    public boolean isAtMiddle() {
        return _middleHall.get() || getPosition()>=MIDDLE_POSITION;
    }

    public boolean isOnSurface() {
        return _downIR.getAverageValue() > Constants.Stilt.DOWN_IR_THRESHOLD;
    }

    public boolean isOnSurfaceLow() {
        return _downIR.getAverageValue() > Constants.Stilt.DOWN_IR_THRESHOLD_LOW;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new DriveStilt(this, _robot.getOI()));
    }
    @Override
    public void updateDashboard() {
        metric("NEOEncoder", getRawNeoEncoder());
        metric("Position", getPosition());
        metric("Retracted", isRetracted());
        metric("Extended", isExtended());
        metric("InMiddle", isAtMiddle());
        metric("IRValue", _downIR.getAverageValue());
        metric("IRVoltage", _downIR.getAverageVoltage());
        metric("OnSurface", isOnSurface());
    }

    public boolean isTilted() {
        return false; // _downIR.getAverageValue() > TILT_THRESHOLD;
    }
}