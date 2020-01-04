package org.frc5687.freddy2020.robot.utils;

import edu.wpi.first.wpilibj.Joystick;

public class Launchpad extends Joystick {
    public Launchpad(int port) {
        super(port);
    }

    public static enum LEDs {
        A(1),
        B(2),
        C(3),
        D(4),
        E(5),
        F(6),
        G(7),
        H(8);

        private final int number;

        LEDs(int number) {
            this.number = number;
        }

        public int getNumber() {
            return number;
        }
    }
}
