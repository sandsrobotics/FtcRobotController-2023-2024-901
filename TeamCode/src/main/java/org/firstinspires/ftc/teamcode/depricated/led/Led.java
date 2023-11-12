//package org.firstinspires.ftc.teamcode.depricated.led;
//
//import android.graphics.Color;
//
//import om.self.ezftc.core.Robot;
//import om.self.ezftc.core.part.LoopedRobotPart;
//
//public class Led extends LoopedRobotPart<LedSettings, LedHardware> {
//
//    public Led(Robot robot, LedHardware hardware, LedSettings settings) {
//        super(robot,"led", settings, hardware);
//    }
//
//    public Led(Robot robot){
//        super(robot,"led", new LedSettings(), new LedHardware());
//    }
//
//    public void setLedStatus(Integer mode) {
//        if(mode == 0){
//            getHardware().leds.setColor(Color.RED);
//        } else if(mode == 1){
//            getHardware().leds.setColor(Color.GREEN);
//        }
//    }
//
//
//
//    @Override
//    public void onRunLoop(short runMode) {
//        if(runMode != 1){
//            getHardware().leds.setColor(1, getSettings().led1Supplier.get());
//            getHardware().leds.setColor(2, getSettings().led2Supplier.get());
//            getHardware().leds.setColor(3, getSettings().led3Supplier.get());
//            getHardware().leds.setColor(4, getSettings().led4Supplier.get());
//            getHardware().leds.setColor(5, getSettings().led5Supplier.get());
//            getHardware().leds.setColor(6, getSettings().led6Supplier.get());
//            getHardware().leds.setColor(7, getSettings().led7Supplier.get());
//            getHardware().leds.setColor(8, getSettings().led8Supplier.get());
//            getHardware().leds.setColor(9, getSettings().led9Supplier.get());
//            getHardware().leds.setColor(10, getSettings().led10Supplier.get());
//            //hardware.leds.setColor(Color.WHITE);
//        } else {
//
//        }
//    }
//
//    @Override
//    public void onStop() {
//
//    }
//}
