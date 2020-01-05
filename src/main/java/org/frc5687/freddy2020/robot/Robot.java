package org.frc5687.freddy2020.robot;

import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import jaci.pathfinder.PathfinderFRC;
import jaci.pathfinder.Trajectory;
import org.frc5687.freddy2020.robot.commands.AutoLaunch;
import org.frc5687.freddy2020.robot.commands.AutoDrivePath;
import org.frc5687.freddy2020.robot.commands.KillAll;
import org.frc5687.freddy2020.robot.commands.SandstormPickup;
import org.frc5687.freddy2020.robot.commands.drive.TwoHatchCargoRocket;
import org.frc5687.freddy2020.robot.commands.drive.TwoHatchCloseAndFarRocket;
import org.frc5687.freddy2020.robot.commands.drive.TwoHatchRocket;
import org.frc5687.freddy2020.robot.subsystems.*;
import org.frc5687.freddy2020.robot.utils.*;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot implements ILoggingSource, IPoseTrackable{

    public static IdentityMode identityMode = IdentityMode.competition;
    private Configuration _configuration;
    private RioLogger.LogLevel _dsLogLevel = RioLogger.LogLevel.warn;
    private RioLogger.LogLevel _fileLogLevel = RioLogger.LogLevel.none;

    private int _updateTick = 0;

    private String _name;
    private OI _oi;
    private AHRS _imu;
    private Limelight _limelight;
    private DriveTrain _driveTrain;
    private Elevator _elevator;
    private PDP _pdp;
    private Arm _arm;
    private Shifter _shifter;
    private Lights _lights;
    private Stilt _stilt;
    private CargoIntake _cargoIntake;
    private HatchIntake _hatchIntake;
    private PoseTracker _poseTracker;
    private AutoChooser _autoChooser;

    private Command _autoCommand;
    private AutoChooser.Mode _mode;
    private AutoChooser.Position _position;
    private long _autoPoll = 0;

    private UsbCamera _driverCamera;

    private boolean _fmsConnected;




    /**
     * This function is setRollerSpeed when the robot is first started up and should be
     * used for any initialization code.
     */
    @Override
    public void robotInit() {
        LiveWindow.disableAllTelemetry();
        loadConfigFromUSB();
        RioLogger.getInstance().init(_fileLogLevel, _dsLogLevel);
        metric("Branch", Version.BRANCH);
        metric("Identity", identityMode.toString());
        info("Starting " + this.getClass().getCanonicalName() + " from branch " + Version.BRANCH);
        info("Robot " + _name + " running in " + identityMode.toString() + " mode");

        // Periodically flushes metrics (might be good to configure enable/disable via USB config file)
        new Notifier(MetricTracker::flushAll).startPeriodic(Constants.METRIC_FLUSH_PERIOD);

        // OI must be first...
        _oi = new OI();
        _imu = new AHRS(SPI.Port.kMXP, (byte) 100);

        try {
            Thread.sleep(2000);
        } catch(Exception e) {

        }
        _imu.zeroYaw();

        // then proxies...
        _lights = new Lights(this);
        _limelight = new Limelight("limelight");
        _pdp = new PDP();

        _autoChooser = new AutoChooser(getIdentityMode()==IdentityMode.competition);
        // Then subsystems....
        _shifter = new Shifter(this);
        _driveTrain = new DriveTrain(this);
        _arm = new Arm(this);
        _elevator = new Elevator(this);
        _stilt = new Stilt(this);
        _cargoIntake = new CargoIntake(this);
        _hatchIntake = new HatchIntake(this);

        // Must be before OI
        _poseTracker = new PoseTracker(this);

        // Must initialize buttons AFTER subsystems are allocated...
        _oi.initializeButtons(this);

        try {
            _driverCamera = CameraServer.getInstance().startAutomaticCapture(0);
            _driverCamera.setResolution(160, 120);
            _driverCamera.setFPS(30);
        } catch (Exception e) {
            DriverStation.reportError(e.getMessage(), true);
        }


        // Initialize the other stuff
        _limelight.disableLEDs();
        _limelight.setStreamingMode(Limelight.StreamMode.PIP_SECONDARY);
        setConfiguration(Configuration.starting);
        //_arm.resetEncoders();
        _arm.enableBrakeMode();
        _elevator.enableBrakeMode();
        _stilt.enableBrakeMode();
    }

    /**
     * This function is called every robot packet, no matter the mode. Use
     * this for items like diagnostics that you want ran during disabled,
     * autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before
     * LiveWindow and SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        updateDashboard();
        _oi.poll();
        update();
    }


    @Override
    public void disabledPeriodic() {
        pollAutoChooser();
    }

    /**
     * This autonomous (along with the chooser code above) shows how to select
     * between different autonomous modes using the dashboard. The sendable
     * chooser code works with the Java SmartDashboard. If you prefer the
     * LabVIEW Dashboard, remove all of the chooser code and uncomment the
     * getString line to get the auto name from the text box below the Gyro
     *
     * <p>You can add additional auto modes by adding additional comparisons to
     * the switch structure below with additional strings. If using the
     * SendableChooser make sure to add them to the chooser code above as well.
     */
    @Override
    public void autonomousInit() {
        _fmsConnected =  DriverStation.getInstance().isFMSAttached();
        _driveTrain.enableBrakeMode();
        _limelight.disableLEDs();
        _limelight.setStreamingMode(Limelight.StreamMode.PIP_SECONDARY);
        AutoChooser.Mode mode = _autoChooser.getSelectedMode();//AutoChooser.Mode.NearAndFarRocket;
        AutoChooser.Position position = _autoChooser.getSelectedPosition();
        if (_autoCommand==null || mode!=_mode || position!=_position) {
            initAutoCommand();
        }
        error("Starting autocommand " + _autoCommand.getClass().getSimpleName());
        _autoCommand.start();
    }

    public void initAutoCommand() {
        _fmsConnected =  DriverStation.getInstance().isFMSAttached();
        AutoChooser.Mode mode = _autoChooser.getSelectedMode();//AutoChooser.Mode.NearAndFarRocket;
        AutoChooser.Position position = _autoChooser.getSelectedPosition();
        // If we already have the command and the mode/position haven't changed, we're done.
        if (_autoCommand != null && mode==_mode && position==position) {
            return;
        }

        // If we already had a command, it's the wrong one!  Clear it and run garbage collection.
        if (_autoCommand!=null) {
            _autoCommand = null;
            System.gc();
        }

        boolean leftSide = position == AutoChooser.Position.LeftPlatform || position == AutoChooser.Position.LeftHAB;

        try {
            switch (mode) {
                case Launch:
                    if ((position == AutoChooser.Position.LeftHAB) || (position == AutoChooser.Position.RightHAB)) {
                        _autoCommand = new AutoLaunch(this);
                    }
                    break;
                case NearAndTopRocket:
                    if ((position != AutoChooser.Position.CenterLeft) && (position != AutoChooser.Position.CenterRight)) {
                        // If we are in the center we can't do rocket hatches!
                        _autoCommand = new TwoHatchRocket(this,
                                position == AutoChooser.Position.LeftHAB || position == AutoChooser.Position.RightHAB,
                                position == AutoChooser.Position.LeftPlatform || position == AutoChooser.Position.LeftHAB);
                    }
                    break;
                case NearAndFarRocket:
                    if ((position != AutoChooser.Position.CenterLeft) && (position != AutoChooser.Position.CenterRight)) {
                        _autoCommand = new TwoHatchCloseAndFarRocket(this,
                                position == AutoChooser.Position.LeftHAB || position == AutoChooser.Position.RightHAB,
                                leftSide
                        );
                    }
                    break;
                case CargoFaceAndNearRocket:

                    _autoCommand = new TwoHatchCargoRocket(this,
                            position == AutoChooser.Position.LeftHAB || position == AutoChooser.Position.RightHAB,
                            position == AutoChooser.Position.CenterLeft || position == AutoChooser.Position.LeftHAB);
                    break;
            }
        } catch (IOException ioe) {

        }
        if (_autoCommand==null) {
            _autoCommand = new SandstormPickup(this);
        }
        _mode = mode;
        _position = position;
        error("autoCommand is " + _autoCommand.getClass().getSimpleName());
    }

    /***
     *  Poll the AutoChooser to see if the values have changed and they've been stable for at least a second...
     */
    private void pollAutoChooser() {
        AutoChooser.Mode mode = _autoChooser.getSelectedMode();//AutoChooser.Mode.NearAndFarRocket;
        AutoChooser.Position position = _autoChooser.getSelectedPosition();
        if (mode!=_mode || position!=_position) {
            // A switch was changed...reset the counter
            _autoPoll = System.currentTimeMillis() + Constants.Auto.AUTOCHOOSER_DELAY;
            return;
        }
        if (System.currentTimeMillis() >= _autoPoll) {
            _autoPoll = Long.MAX_VALUE;
            initAutoCommand();
        }


    }

    public void teleopInit() {
        _fmsConnected =  DriverStation.getInstance().isFMSAttached();
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
        ourPeriodic();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
        ourPeriodic();
    }

    private void ourPeriodic() {
        // Example of starting a new row of metrics for all instrumented objects.
        MetricTracker.newMetricRowAll();

        if (_oi.isKillAllPressed()) {
            new KillAll(this).start();
        }
        Scheduler.getInstance().run();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        Scheduler.getInstance().run();
    }

    @Override
    public void disabledInit() {
        //_limelight.disableLEDs();
        RioLogger.getInstance().forceSync();
        RioLogger.getInstance().close();
        _arm.enableCoastMode();
        _elevator.enableCoastMode();
        _stilt.enableCoastMode();
    }


    public void updateDashboard() {
        _updateTick++;
        if (_updateTick >= (_fmsConnected ? Constants.TICKS_PER_UPDATE_COMP : Constants.TICKS_PER_UPDATE)) {
            _updateTick = 0;
            _oi.updateDashboard();
            _driveTrain.updateDashboard();
            _limelight.updateDashboard();
            _arm.updateDashboard();
            _elevator.updateDashboard();
            _pdp.updateDashboard();
            _shifter.updateDashboard();
            _lights.updateDashboard();
            _stilt.updateDashboard();
            _cargoIntake.updateDashboard();
            _hatchIntake.updateDashboard();
            _autoChooser.updateDashboard();
            metric("imu/yaw", _imu.getYaw());
            metric("imu/pitch", _imu.getPitch());
            metric("imu/roll", _imu.getRoll());
            metric("Memory", Runtime.getRuntime().freeMemory());
        }
    }


    private void loadConfigFromUSB() {    String output_dir = "/U/"; // USB drive is mounted to /U on roboRIO
        try {
            String usbDir = "/U/"; // USB drive is mounted to /U on roboRIO
            String configFileName = usbDir + "frc5687.cfg";
            File configFile = new File(configFileName);
            FileReader reader = new FileReader(configFile);
            BufferedReader bufferedReader = new BufferedReader(reader);

            String line;
            while ((line = bufferedReader.readLine())!=null) {
                processConfigLine(line);
            }

            bufferedReader.close();
            reader.close();
        } catch (Exception e) {
            identityMode = IdentityMode.competition;
        }
    }

    private void processConfigLine(String line) {
        try {
            if (line.startsWith("#")) { return; }
            String[] a = line.split("=");
            if (a.length==2) {
                String key = a[0].trim().toLowerCase();
                String value = a[1].trim();
                switch (key) {
                    case "name":
                        _name = value;
                        metric("name", _name);
                        break;
                    case "mode":
                        identityMode = IdentityMode.valueOf(value.toLowerCase());
                        metric("mode", identityMode.toString());
                        break;
                    case "fileloglevel":
                        _fileLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        metric("fileLogLevel", _fileLogLevel.toString());
                        break;
                    case "dsloglevel":
                        _dsLogLevel = RioLogger.LogLevel.valueOf(value.toLowerCase());
                        metric("dsLogLevel", _dsLogLevel.toString());
                        break;
                }
            }
        } catch (Exception e) {

        }
    }
    public void setConfiguration(Configuration configuration) {
        _configuration = configuration;
        //_oi.setConfigurationLEDs(configuration);
    }

    public Configuration getConfiguration() {
        return _configuration;
    }

    private boolean _wereLEDsOn = false;
    private boolean _trackingRetrieveHatch = false;
    private boolean _trackingScoreHatch = false;
    private boolean _wasShocked = false;

    private void update() {

        double timeLeft = DriverStation.getInstance().getMatchTime();
        if (DriverStation.getInstance().isOperatorControl() && timeLeft <= Constants.FINAL_WARNING
        && _configuration!=Configuration.climbing && _configuration != Configuration.parked) {
            _lights.setColor(Constants.Lights.PULSING_YELLOW, 0);
            _oi.setConsoleColor(false, true, true);
        }

        if (_hatchIntake.isShockTriggered()) {
            if (!_wasShocked) {
                _oi.pulseDriver(4);
                _oi.pulseOperator(4);
            }
            _wasShocked = true;
        } else {
            _wasShocked = false;
        }
        switch (_configuration) {
            case starting:
                _oi.setConsoleColor(false, false, false);
                if (DriverStation.getInstance().getAlliance()== DriverStation.Alliance.Red) {
                    _lights.setColor(Constants.Lights.BEATING_RED, 0);
                } else if (DriverStation.getInstance().getAlliance()== DriverStation.Alliance.Blue) {
                    _lights.setColor(Constants.Lights.BEATING_BLUE, 0);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_YELLOW, 0);
                }
                break;
            case hatch:
                // In hatch mode,
                //  - intake pointed
                //    - limelight on - started while no hatch
                //    - target sighted, started while no hatch
                //    - target centered, started while no hatch
                //    - no hatch - pulsing purple
                //    - hatch detected - solid purple
                //  - intake gripped -
                //    - limelight on - started while had hatch
                //    - target sighted, started while had hatch
                //    - target centered, started while had hatch
                //    - has hatch - solid purple
                //    - no hatch - pale puple
                if (_hatchIntake.isPointed()) {
                    _lights.setColor(Constants.Lights.SOLID_WHITE, 0);
                    setDashLEDs(true);
                    _oi.setConsoleColor(true, true, true);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_YELLOW, 0);
                    setDashLEDs(false);
                    _oi.setConsoleColor(false, false, false);
                }
                break;
            case cargo:
                // Intake running?
                //   Has cargo? SOLID ORANGE
                //   Targeting?
                //   Detected?
                //   Centered?
                //   No?  PULSING ORANGE
                // Intake not running
                //   Has cargo? SOLID ORANGE
                //   No cargo? PALE
                if (_cargoIntake.isEjecting()) {
                    _lights.setColor(Constants.Lights.SOLID_WHITE, 0);
                    _oi.setConsoleColor(true, true, true);
                } else if (_cargoIntake.isBallDetected()) {
                    _lights.setColor(Constants.Lights.SOLID_GREEN, 0);
                    _oi.setConsoleColor(false, true, false);
                } else if (_cargoIntake.isIntaking()) {
                    _lights.setColor(Constants.Lights.PULSING_RED, 0);
                    _oi.setConsoleColor(true, false, false);
                } else {
                    _lights.setColor(Constants.Lights.SOLID_PURPLE, 0);
                    _oi.setConsoleColor(false, false, false);
                }
                break;
            case climbing:
                _oi.setConsoleColor(false, false, false);
                _lights.setColor(Constants.Lights.WHITE_SHOT, 0);
                break;
            case parked:
                _lights.setColor(Constants.Lights.CONFETTI, 0);
                _oi.setConsoleColor(false, true, false);
                break;
        }
        _wereLEDsOn = _limelight.areLEDsOn();
        if (!_wereLEDsOn) {
            _trackingScoreHatch = false;
            _trackingRetrieveHatch = false;
        }
    }

    private void setDashLEDs(boolean val) {
        SmartDashboard.putBoolean("DB/LED 0", val);
        SmartDashboard.putBoolean("DB/LED 1", val);
        SmartDashboard.putBoolean("DB/LED 2", val);
        SmartDashboard.putBoolean("DB/LED 3", val);
    }

    @Override
    public void error(String message) {
        RioLogger.error(this, message);
    }

    @Override
    public void warn(String message) {
        RioLogger.warn(this, message);
    }

    @Override
    public void info(String message) {
        RioLogger.info(this, message);
    }

    @Override
    public void debug(String message) {
        RioLogger.debug(this, message);
    }

    public OI getOI() {
        return _oi;
    }
    public AHRS getIMU() { return _imu; }
    public DriveTrain getDriveTrain() { return _driveTrain; }
    public Limelight getLimelight() {
        return _limelight;
    }
    public PDP getPDP() { return _pdp; }
    public Arm getArm() { return _arm; }
    public Elevator getElevator() { return _elevator; }
    public Shifter getShifter() { return _shifter; }
    public Lights getLights() { return _lights; }
    public Stilt getStilt() { return _stilt; }
    public CargoIntake getCargoIntake() { return _cargoIntake;}
    public HatchIntake getHatchIntake() { return _hatchIntake;}
    public PoseTracker getPoseTracker() { return _poseTracker; }

    @Override
    public Pose getPose() {
        return new BasicPose(_imu.getYaw(), _driveTrain.getLeftDistance(), _driveTrain.getRightDistance(), _driveTrain.getDistance());
    }


    public enum IdentityMode {
        competition(0),
        practice(1),
        programming(2);

        private int _value;

        IdentityMode(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public enum Configuration {
        starting(0),
        hatch(1),
        cargo(2),
        climbing(3),
        parked(4);

        private int _value;

        Configuration(int value) {
            this._value = value;
        }

        public int getValue() {
            return _value;
        }
    }

    public IdentityMode getIdentityMode() {
        return identityMode;
    }

    public void metric(String name, boolean value) {
        SmartDashboard.putBoolean(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, String value) {
        SmartDashboard.putString(getClass().getSimpleName() + "/" + name, value);
    }

    public void metric(String name, double value) {
        SmartDashboard.putNumber(getClass().getSimpleName() + "/" + name, value);
    }

    public static double pickConstant(double competitionValue, double practiceValue) {
        return identityMode == IdentityMode.practice ? practiceValue : competitionValue;
    }

    public static long pickConstant(long competitionValue, long practiceValue) {
        return identityMode == IdentityMode.practice ? practiceValue : competitionValue;
    }

}
