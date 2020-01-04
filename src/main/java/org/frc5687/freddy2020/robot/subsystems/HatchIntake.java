package org.frc5687.freddy2020.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import org.frc5687.freddy2020.robot.Robot;
import org.frc5687.freddy2020.robot.RobotMap;
import org.frc5687.freddy2020.robot.commands.intake.IdleHatchIntake;
import org.frc5687.freddy2020.robot.utils.HallEffect;
import org.frc5687.freddy2020.robot.utils.LimitSwitch;

public class HatchIntake extends OutliersSubsystem {


    private Robot _robot;
    private DoubleSolenoid _clawSolenoid;
    private DoubleSolenoid _wristSolenoid;
    private LimitSwitch _hatchDetectionLimit;
    private HallEffect _shockHall;

    public HatchIntake(Robot robot) {
        _robot = robot;
        _clawSolenoid = new DoubleSolenoid(RobotMap.PCM.CLAW_OPEN, RobotMap.PCM.CLAW_CLOSE);
        _wristSolenoid = new DoubleSolenoid(RobotMap.PCM.CLAW_WRIST_UP, RobotMap.PCM.CLAW_WRIST_DOWN);
        _hatchDetectionLimit = new LimitSwitch(RobotMap.DIO.HATCH_DETECTION_LIMIT);
        _shockHall = new HallEffect(RobotMap.DIO.SHOCK_HALL);
    }

    @Override
    public void updateDashboard() {
        metric("Wrist", _wristSolenoid.get().name());
        metric("Claw", _clawSolenoid.get().name());
        metric("HatchDetected", isHatchDetected());
        metric("ShockTriggered", isShockTriggered());
    }


    public void gripClaw(){
        _clawSolenoid.set(DoubleSolenoid.Value.kForward);
    }

    public void pointClaw() {
        _clawSolenoid.set(DoubleSolenoid.Value.kReverse);
    }

    public void raiseWrist() { _wristSolenoid.set(DoubleSolenoid.Value.kReverse); }

    public void lowerWrist() { _wristSolenoid.set(DoubleSolenoid.Value.kForward); }

    public void releaseWrist() { _wristSolenoid.set(DoubleSolenoid.Value.kOff); }

    @Override
    protected void initDefaultCommand() {
        setDefaultCommand(new IdleHatchIntake(this));
    }

    public boolean isDown() {
        return _wristSolenoid.get() == DoubleSolenoid.Value.kForward;
    }

    public boolean isUp() {
        return _wristSolenoid.get() == DoubleSolenoid.Value.kReverse;
    }

    public boolean isHatchDetected() { return false; }

    public boolean isPointed() {
            if(_clawSolenoid.get() == DoubleSolenoid.Value.kReverse){
            return true;
        }
            return false;
    }

    public boolean isShockTriggered() { return _shockHall.get(); }
}

