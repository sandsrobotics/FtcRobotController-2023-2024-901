package org.firstinspires.ftc.teamcode.parts.led;

import android.graphics.Color;
import org.apache.commons.lang3.ObjectUtils;
//import org.openftc.i2cdrivers.QwiicLEDStick;
import org.firstinspires.ftc.teamcode.lib.QwiicLEDStick;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

public class Led extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {

    QwiicLEDStick leds;

    public Led(Robot robot){
        super(robot, "Leds");
    }

    @Override
    public void onRun() {

    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {
        leds = parent.opMode.hardwareMap.get(QwiicLEDStick.class, "statLed");
    }

    @Override
    public void onStart() {
        leds.setBrightness(1);//max of 31
        leds.setColor(Color.WHITE);
        leds.setColor(7,Color.GREEN);
    }

    @Override
    public void onStop() {
        leds.setColor(Color.BLACK);
        leds.setBrightness(0);
        leds.turnAllOff();
    }
}
