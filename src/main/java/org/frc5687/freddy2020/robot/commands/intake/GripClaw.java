package org.frc5687.freddy2020.robot.commands.intake;

import edu.wpi.first.wpilibj.DriverStation;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.commands.OutliersCommand;
import org.frc5687.freddy2020.robot.subsystems.CargoIntake;
import org.frc5687.freddy2020.robot.subsystems.HatchIntake;

public class GripClaw extends OutliersCommand {
    public HatchIntake _hatchIntake;
    private long _endTime;
    private long _delay;

    public GripClaw(HatchIntake hatchIntake, long delay) {
        _delay = delay;
        _hatchIntake = hatchIntake;
        requires(_hatchIntake);
    }

    public GripClaw(HatchIntake hatchIntake) {
        this(hatchIntake, Constants.Intake.CLOSE_CLAW_MILLI_SEC);
//        DriverStation.reportError("GripingCaw", true);
    }

    @Override
    protected void initialize() {
        _hatchIntake.gripClaw();
        _endTime = System.currentTimeMillis() + _delay;
    }

    @Override
    protected boolean isFinished() {
        return System.currentTimeMillis() > _endTime;
    }

    @Override
    protected void execute(){
    }
}
