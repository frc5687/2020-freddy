package org.frc5687.freddy2020.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.command.ConditionalCommand;
import org.frc5687.freddy2020.robot.commands.*;
import org.frc5687.freddy2020.robot.commands.drive.*;
import org.frc5687.freddy2020.robot.commands.intake.*;
import org.frc5687.freddy2020.robot.subsystems.Elevator;
import org.frc5687.freddy2020.robot.subsystems.Shifter;
import org.frc5687.freddy2020.robot.utils.*;

import static org.frc5687.freddy2020.robot.utils.Helpers.applyDeadband;
import static org.frc5687.freddy2020.robot.utils.Helpers.applySensitivityFactor;

public class OI extends OutliersProxy {
    protected Gamepad _driverGamepad;
    protected Joystick _driverLeftjoystick;
    protected Joystick _driverRightjoystick;
    protected Gamepad _operatorGamepad;
    protected Launchpad _launchpad;

    private Button _operatorRightTrigger;
    private Button _operatorLeftTrigger;
    private Button _driverRightTrigger;
    private Button _driverLeftTrigger;

    private Button _driverRightStickButton;
    private AxisButton _driverRightYAxisUpButton;
    private AxisButton _driverRightYAxisDownButton;


    private Button _operatorAButton;
    private Button _operatorBButton;
    private Button _operatorYButton;
    private Button _operatorXButton;

    private Button _operatorRightBumper;
    private Button _operatorLeftBumper;

    private Button _driverAButton;
    private Button _driverBButton;
    private Button _driverXButton;
    private Button _driverYButton;

    private Button _driverRightBumper;
    private Button _driverLeftBumper;

    private Button _operatorStartButton;
    private Button _operatorBackButton;
    private Button _driverStartButton;
    private Button _driverBackButton;

    private Button _operatorUpButton;
    private Button _operatorDownButton;
    private Button _driverUpButton;
    private Button _driverDownButton;

    private AxisButton _operatorRightXAxisRightButton;
    private AxisButton _operatorRightXAxisLeftButton;

    private Button _operatorRightStickButton;

    private POV _operatorPOV;

    private JoystickLight _hatchModeLED;
    private JoystickLight _cargoModeLED;
    private JoystickLight _hatchIntakeLED;
    private JoystickLight _cargoIntakeLED;
    private JoystickLight _targetLeftLED;
    private JoystickLight _targetCenteredLED;
    private JoystickLight _targetRightLED;

    private Button _driverLeftButton;
    private Button _driverRightButton;
    private Button _driverTrigger;
    private Button _driverHighButton;
    private Button _driverLowButton;
    private Button _driverStartingButton;

    public OI(){
        _driverGamepad = new Gamepad(0);
        _operatorGamepad = new Gamepad(1);
        _driverLeftjoystick = new Joystick(2);
        _driverRightjoystick = new Joystick(3);
        try {
            _launchpad = new Launchpad(4);
        } catch (Exception e) {
        }

        _operatorRightTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _operatorLeftTrigger = new AxisButton(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _driverRightTrigger = new AxisButton(_driverGamepad, Gamepad.Axes.RIGHT_TRIGGER.getNumber(),Constants.OI.AXIS_BUTTON_THRESHHOLD);
        _driverLeftTrigger = new AxisButton(_driverGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber(), Constants.OI.AXIS_BUTTON_THRESHHOLD);

        _operatorRightBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _operatorLeftBumper = new JoystickButton(_operatorGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());

        _operatorRightStickButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _driverRightBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_BUMPER.getNumber());
        _driverLeftBumper = new JoystickButton(_driverGamepad, Gamepad.Buttons.LEFT_BUMPER.getNumber());

        _operatorAButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.A.getNumber());
        _operatorBButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.B.getNumber());
        _operatorYButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.Y.getNumber());
        _operatorXButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.X.getNumber());

        _driverAButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.A.getNumber());
        _driverBButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.B.getNumber());
        _driverXButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.X.getNumber());
        _driverYButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.Y.getNumber());


        _driverRightStickButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.RIGHT_STICK.getNumber());

        _driverRightYAxisUpButton = new AxisButton(_driverRightjoystick, _driverRightjoystick.getYChannel(), -.75);
        _driverRightYAxisDownButton = new AxisButton(_driverGamepad,Gamepad.Axes.RIGHT_Y.getNumber(), 0.75);

        _operatorStartButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.START.getNumber());
        _operatorBackButton = new JoystickButton(_operatorGamepad, Gamepad.Buttons.BACK.getNumber());

        _driverStartButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.START.getNumber());
        _driverBackButton = new JoystickButton(_driverGamepad, Gamepad.Buttons.BACK.getNumber());

        _operatorUpButton = new JoystickButton(_operatorGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());
        _operatorDownButton = new JoystickButton(_operatorGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());

        _driverUpButton = new JoystickButton(_driverGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());
        _driverDownButton = new JoystickButton(_driverGamepad, Gamepad.Axes.D_PAD_VERTICAL.getNumber());

        _operatorRightXAxisLeftButton = new AxisButton(_operatorGamepad,Gamepad.Axes.RIGHT_X.getNumber(), -.5);
        _operatorRightXAxisRightButton = new AxisButton(_operatorGamepad, Gamepad.Axes.RIGHT_X.getNumber(), .5);

        _driverTrigger = new JoystickButton(_driverRightjoystick, 1);
        _driverLeftButton = new JoystickButton(_driverRightjoystick,2);
        _driverRightButton = new JoystickButton(_driverRightjoystick, 3);
        _driverHighButton = new  JoystickButton(_driverLeftjoystick, 3);
        _driverLowButton = new JoystickButton(_driverLeftjoystick, 2);
        _driverStartingButton = new JoystickButton(_driverRightjoystick, 4);

        // _operatorPOV = new POV();
    }
    public void initializeButtons(Robot robot){

        _driverRightButton.whenPressed(new SafeguardCommand(robot, new AutoClimb(robot.getStilt(), robot.getArm(), robot.getDriveTrain(), robot.getCargoIntake(), robot.getHatchIntake(), robot, true), -30));
        _driverLeftButton.whenPressed(new SafeguardCommand(robot, new AutoClimb(robot.getStilt(), robot.getArm(), robot.getDriveTrain(), robot.getCargoIntake(), robot.getHatchIntake(), robot, false), -30));

        _driverLowButton.whenPressed(new Shift(robot.getDriveTrain(), robot.getShifter(), Shifter.Gear.LOW, false));
        _driverHighButton.whenPressed(new Shift(robot.getDriveTrain(), robot.getShifter(), Shifter.Gear.HIGH, false));

        _driverLeftTrigger.whenPressed(new Eject(robot));
        _driverRightTrigger.whenPressed(new Intake(robot));


        _driverAButton.whenPressed(new ConditionalCommand(
                new AutoAlign(robot.getDriveTrain(), robot.getIMU(), 180, 1,1000,5.0,"Aligning to Human Player Station")) {
            @Override
            protected boolean condition() {
                return robot.getConfiguration()!=Robot.Configuration.climbing && robot.getConfiguration()!=Robot.Configuration.parked;
            }
        });

        _operatorLeftBumper.whenPressed(new ConditionalCommand(new SandstormPickup(robot), new HatchMode(robot)) {
            @Override
            protected boolean condition() {
                return DriverStation.getInstance().isAutonomous() && robot.getConfiguration() == Robot.Configuration.starting;
            }
        });

        _operatorRightBumper.whenPressed(new CargoMode(robot));

//        _operatorBackButton.whenPressed(new AutoLaunch(robot));
        _operatorStartButton.whenPressed(new StartingConfiguration(robot));
        _driverStartingButton.whenPressed(new StartingConfiguration(robot));

        _operatorRightTrigger.whenPressed(new IntakeCargo(robot));
        _operatorLeftTrigger.whileHeld(new HoldClawOpen(robot));

        _operatorRightXAxisLeftButton.whenPressed(new CargoIntakeDown(robot.getCargoIntake()));
        _operatorRightXAxisRightButton.whenPressed(new CargoIntakeUp(robot.getCargoIntake()));


        _operatorAButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch1, Elevator.MotionMode.Ramp, this, 0.0));
        _operatorBButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch2, Elevator.MotionMode.Ramp, this, 0.0));
        _operatorYButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.Hatch3, Elevator.MotionMode.Ramp, this, 0.0));
        _operatorXButton.whenPressed(new MoveElevatorToSetPoint(robot.getElevator(), Elevator.Setpoint.HPMode, Elevator.MotionMode.Ramp, this, 0.0));

    }

    public boolean isAutoTargetPressed() {
        return _driverTrigger.get();
    }
    public double getDriveSpeed() {
        double speed = -getSpeedFromAxis(_driverLeftjoystick, _driverLeftjoystick.getYChannel());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }

    public double getDriveRotation() {
        double speed = getSpeedFromAxis(_driverRightjoystick, _driverRightjoystick.getXChannel());
        speed = applyDeadband(speed, Constants.DriveTrain.DEADBAND);
        return speed;
    }
    public double getArmSpeed() {
        return 0;
//        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_Y.getNumber()) * Constants.Arm.MAX_DRIVE_SPEED;
//        speed = applyDeadband(speed, Constants.Arm.DEADBAND);
//        return applySensitivityFactor(speed, Constants.Arm.SENSITIVITY);
    }
    public double getRollerSpeed() {
        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_X.getNumber()) * Constants.Intake.MAX_ROLLER_SPEED;
        speed = applyDeadband(speed, Constants.Intake.DEADBAND);
        return applySensitivityFactor(speed, Constants.Intake.SENSITIVITY);
    }
    public double getElevatorSpeed() {
//        return 0;
        double speed = -getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber()) * Constants.Elevator.MAX_SPEED;
        speed = applyDeadband(speed, Constants.Elevator.DEADBAND);
        return applySensitivityFactor(speed, Constants.Elevator.SENSITIVITY);
    }
    public double getStiltSpeed() {
        return 0;
//        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.RIGHT_Y.getNumber())*Constants.Stilt.MAX_UP_SPEED;
//        speed = applyDeadband(speed, Constants.Stilt.DEADBAND);
//        return speed;
    }

    public double getWheelieSpeed() {
        return 0;
//        double speed = getSpeedFromAxis(_operatorGamepad, Gamepad.Axes.LEFT_TRIGGER.getNumber());
//        speed = applyDeadband(speed, Constants.Stilt.DEADBAND);
//        return speed;
    }


    public int getOperatorPOV() {
        return POV.fromWPILIbAngle(0, _operatorGamepad.getPOV()).getDirectionValue();
    }
    public int getDriverPOV() {
        return POV.fromWPILIbAngle(0, _driverLeftjoystick.getPOV()).getDirectionValue();
    }

    protected double getSpeedFromAxis(Joystick gamepad, int axisNumber) {
        return gamepad.getRawAxis(axisNumber);
    }

    @Override
    public void updateDashboard() {

    }

    private int _driverRumbleCount = 0;
    private int _operatorRumbleCount = 0;
    private long _driverRumbleTime = System.currentTimeMillis();
    private long _operatorRumbleTime = System.currentTimeMillis();

    public void pulseDriver(int count) {
        // Check to see if we are already rumbling!
        if (_driverRumbleCount > 0) { return; }
        _driverRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
        _driverRumbleCount = count * 2;
    }

    public void pulseOperator(int count) {
        // Check to see if we are already rumbling!
        if (_operatorRumbleCount > 0) { return; }
        _operatorRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
        _operatorRumbleCount = count * 2;
    }

    public void poll() {
        if (_driverRumbleCount > 0) {
            _driverGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, _driverRumbleCount % 2 == 0 ? 0 : 1);
            _driverGamepad.setRumble(GenericHID.RumbleType.kRightRumble, _driverRumbleCount % 2 == 0 ? 0 : 1);
            if (System.currentTimeMillis() > _driverRumbleTime) {
                _driverRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
                _driverRumbleCount--;
            }
        } else {
            _driverGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            _driverGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }

        if (_operatorRumbleCount > 0) {
            _operatorGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, _operatorRumbleCount % 2 == 0 ? 0 : 1);
            _operatorGamepad.setRumble(GenericHID.RumbleType.kRightRumble, _operatorRumbleCount % 2 == 0 ? 0 : 1);
            if (System.currentTimeMillis() > _operatorRumbleTime) {
                _operatorRumbleTime = System.currentTimeMillis() + Constants.OI.RUMBLE_PULSE_TIME;
                _operatorRumbleCount--;
            }
        } else {
            _operatorGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            _operatorGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }
    }

    public boolean isCreepPressed() {
        return  _driverRightStickButton.get();
    }

    public boolean isWheelieForwardPressed() {
        return  _driverYButton.get();
    }

    public boolean isKillAllPressed() {
        int operatorPOV = getOperatorPOV();
        int driverPOV = getDriverPOV();

        return driverPOV == Constants.OI.KILL_ALL || operatorPOV == Constants.OI.KILL_ALL;
    }

    public boolean isOverridePressed() {
        int operatorPOV = getOperatorPOV();
        int driverPOV = getDriverPOV();

        return driverPOV == Constants.OI.OVERRIDE || operatorPOV == Constants.OI.OVERRIDE;
    }

    public void setConsoleColor(boolean red, boolean green, boolean blue) {
        if (_launchpad==null) { return; }
        try {
            _launchpad.setOutput(Constants.OI.RED_CHANNEL, red);
            _launchpad.setOutput(Constants.OI.GREEN_CHANNEL, green);
            _launchpad.setOutput(Constants.OI.BLUE_CHANNEL, blue);
        } catch (Exception e) {
        }
    }
}

