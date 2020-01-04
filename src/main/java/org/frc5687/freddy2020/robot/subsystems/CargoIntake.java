package org.frc5687.freddy2020.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.RobotMap;
import org.frc5687.freddy2020.robot.commands.intake.RunIntake;
import org.frc5687.freddy2020.robot.utils.Helpers;

import static org.frc5687.freddy2020.robot.Constants.Intake.*;

public class CargoIntake extends OutliersSubsystem {


    private Robot _robot;
    private DoubleSolenoid _wristSolenoid;
    private TalonSRX _roller;
    private RollerMode _rollerMode;
    private boolean _forceOn;
    private AnalogInput _ballIR;

    private double _rollerSpeed;

    public CargoIntake(Robot robot) {
        _robot = robot;
        _wristSolenoid = new DoubleSolenoid(RobotMap.PCM.WRIST_UP, RobotMap.PCM.WRIST_DOWN);
        try {
            _roller = new TalonSRX(RobotMap.CAN.TALONSRX.ROLLER);
            _roller.configPeakOutputForward(HIGH_POW, 0);
            _roller.configPeakOutputReverse(LOW_POW,0);
            _roller.configNominalOutputForward(0.0, 0);
            _roller.configNominalOutputReverse(0.0, 0);
            _roller.setInverted(Constants.Intake.MOTOR_INVERTED);
            _roller.setNeutralMode(NeutralMode.Brake);
        } catch (Exception e) {
            error("Unable to allocate roller controller: " + e.getMessage());
        }
        _ballIR = new AnalogInput(RobotMap.Analog.BALL_IR);
    }

    @Override
    public void updateDashboard() {
        metric("IRValue", _ballIR.getValue());
        metric ("BallDetected", isBallDetected());
        metric("ForceOn", _forceOn);
    }

    public void startRoller() {
        setRollerSpeed(ROLLER_SPEED);
        _forceOn = true;
    }

    public void stopRoller() {
        setRollerSpeed(0);
        _forceOn = false;
    }

    public void raiseWrist() {
        _wristSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void lowerWrist() {
        _wristSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void releaseWrist() { _wristSolenoid.set(DoubleSolenoid.Value.kOff); }

    public void setRollerSpeed(double speed) {
        speed = Helpers.limit(speed, Constants.Intake.MAX_ROLLER_SPEED);
        if (_roller == null) {
            return;
        }
        run(speed);
    }

    public void run(double speed) {
        if (_forceOn) { speed = ROLLER_SPEED; }
        metric("RollerSpeed", speed);
        _rollerSpeed = speed;
        _roller.set(ControlMode.PercentOutput, speed);
    }

    public boolean isBallDetected() {
        return _ballIR.getValue() > Constants.Intake.CARGO_DETECTED_THRESHOLD;
    }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new RunIntake(_robot, this));
    }

    public void setRollerMode(RollerMode rollerMode) {
        _rollerMode = rollerMode;
    }
    public RollerMode getRollerMode() {
        return _rollerMode;
    }

    public boolean isDown() {
        return _wristSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    public boolean isUp() {
        return _wristSolenoid.get() == DoubleSolenoid.Value.kReverse;
    }

    public boolean isIntaking() {
        return _rollerSpeed > 0;
    }

    public boolean isEjecting() {
        return _rollerSpeed < 0;
    }


    public enum RollerMode {
        RUNNING(0),
        WAITING(1),
        DONE(2);

        private int _value;

        RollerMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }

    }

}

