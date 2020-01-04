package org.frc5687.freddy2020.robot.utils;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import org.frc5687.freddy2020.robot.Constants;
import org.frc5687.freddy2020.robot.RobotMap;

/**
 * Created by Ben Bernard on 2/2/2018.
 */
public class AutoChooser extends OutliersProxy {
    private RotarySwitch _positionSwitch;
    private RotarySwitch _modeSwitch;

    public AutoChooser(boolean isCompetitionBot) {


        if (isCompetitionBot) {
            _positionSwitch = new RotarySwitch(RobotMap.Analog.POSITION_SWITCH,  Constants.RotarySwitch.TOLERANCE, 0.07692, 0.15384, 0.23076, 0.30768, 0.3846, 0.46152, 0.53844, 0.61536, 0.69228, 0.7692, 0.84612, 0.92304);
            _modeSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH,  Constants.RotarySwitch.TOLERANCE, 0.07692, 0.15384, 0.23076, 0.30768, 0.3846, 0.46152, 0.53844, 0.61536, 0.69228, 0.7692, 0.84612, 0.92304);
        } else {
            _positionSwitch = new RotarySwitch(RobotMap.Analog.POSITION_SWITCH, Constants.RotarySwitch.TOLERANCE * 3, .092, .235, .505, .680, .823, .958);
            _modeSwitch = new RotarySwitch(RobotMap.Analog.MODE_SWITCH, Constants.RotarySwitch.TOLERANCE, .09, .17, .23, .31, .5, .59, .68, .75, .82, .91, .96);
        }
    }


    public Position getSelectedPosition(){
        int raw = _positionSwitch.get();
        if (raw >= Position.values().length) { raw = 0; }
        return Position.values()[raw];
    }

    public Mode getSelectedMode(){
        int raw = _modeSwitch.get();
        if (raw >= Mode.values().length) { raw = 0; }
        try {
            return Mode.values()[raw];
        } catch(Exception e){
                return Mode.StayPut;
        }
    }


    public void updateDashboard(){
        metric("Label/Position", getSelectedPosition().getLabel());
        metric("Label/Mode", getSelectedMode().getLabel());
        metric("Raw/Position", _positionSwitch.getRaw());
        metric("Raw/Mode", _modeSwitch.getRaw());
        metric("Numeric/Position", _positionSwitch.get());
        metric("Numeric/Mode", _modeSwitch.get());
  }

  public enum Position {
        LeftHAB(0, "Left HAB"),
        LeftPlatform(1, "Left Platform"),
        CenterLeft(2, "Center Left"),
        CenterRight(3, "Center Right"),
        RightPlatform(4, "Right Platform"),
        RightHAB(5, "Right HAB");

        private String _label;
        private int _value;

        Position(int value, String label) {
            _value = value;
            _label = label;
        }

        public int getValue() { return _value; }
        public String getLabel() { return _label; }
  }

    public enum Mode {
        StayPut(0, "Stay Put"),
        Launch(1, "Launch"),
        CargoFace(2, "Cargo Face"),
        CargoSide(3, "Cargo Side"),
        NearRocket(4, "Near Rocket"),
        NearAndTopRocket(5, "Near and Top Rocket"),
        NearAndFarRocket(6, "Near and Far Rocket"),
        CargoFaceAndNearRocket(7, "Cargo Face and Near Rocket");

        private String _label;
        private int _value;

        Mode(int value, String label) {
            _value = value;
            _label = label;
        }

        public int getValue() { return _value; }
        public String getLabel() { return _label; }

    }
}
