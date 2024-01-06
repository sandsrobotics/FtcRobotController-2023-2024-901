package org.firstinspires.ftc.teamcode.parts.led;

import android.graphics.Color;
import org.apache.commons.lang3.ObjectUtils;
//import org.openftc.i2cdrivers.QwiicLEDStick;
import org.firstinspires.ftc.teamcode.lib.QwiicLEDStick;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

public class Led extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {

    public QwiicLEDStick leds;

    boolean bottomOn = false;
    boolean topOn = false;

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
        leds.turnAllOff();
    }

    @Override
    public void onStop() {
        leds.setColor(Color.BLACK);
        leds.turnAllOff();
    }

    public void setBottomGroup(boolean turnOn){
        if (turnOn != bottomOn) {
            bottomOn = turnOn;
            if (turnOn) leds.setColorGroup(0, 5, Color.BLUE);
            else leds.setColorGroup(0, 5, Color.BLACK);
        }
    }

    public void setTopGroup(boolean turnOn){
        if (turnOn != topOn) {
            topOn = turnOn;
            if (turnOn) leds.setColorGroup(5, 5, Color.GREEN);
            else leds.setColorGroup(5, 5, Color.BLACK);
        }
    }
}
