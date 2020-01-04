package org.frc5687.freddy2020.robot.utils;

import edu.wpi.first.wpilibj.DigitalInput;

public class HallEffect extends DigitalInput {

    public HallEffect(int channel){
        super(channel);
    }

    public boolean get() { return !super.get();}

}


