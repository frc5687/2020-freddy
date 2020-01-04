package org.frc5687.freddy2020.robot.utils;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Notifier;

import java.util.ArrayList;

public class JoystickLight {

    private static int _step = 0;
    private static ArrayList<JoystickLight> _lights = new ArrayList<>(10);
    private final Joystick m_joystick;
    private final int m_outputNumber;
    private boolean m_value = false;
    private State _state = State.off;

    /**
     * @param joystick     The Joystick object that has the lights (e.g. Joystick, Launchpad, etc)
     * @param outputNumber The output number
     */
    public JoystickLight(Joystick joystick, int outputNumber) {
        m_joystick = joystick;
        m_outputNumber = outputNumber;
        _lights.add(this);
    }
/**
 * Gets the value of the joystick button.
 *
 * @return The value of the joystick button
 */
    public boolean get() {
        return m_value;
    }

    public void set(boolean value) {
        _state = value ? State.on : State.off;
    }

    public void set(State state) {
        _state = state;
        cycle();
    }

    protected void cycle() {
        switch (_state) {
            case off:
                m_joystick.setOutput(m_outputNumber, false);
                break;
            case on:
                m_joystick.setOutput(m_outputNumber, false);
                break;
            case slow_blink:
                m_joystick.setOutput(m_outputNumber, _step < 2);
                break;
            case fast_blink:
                m_joystick.setOutput(m_outputNumber, _step == 0 || _step==2);
                break;
        }
    }

    public enum State {
        off,
        on,
        slow_blink,
        fast_blink
    }

    public static void initialize(double period) {
        new Notifier(JoystickLight::poll).startPeriodic(period);
    }

    public static void poll() {
        _step++;
        if (_step>3) {_step=0;}
        for(var light : _lights) { light.cycle(); }
    }
}