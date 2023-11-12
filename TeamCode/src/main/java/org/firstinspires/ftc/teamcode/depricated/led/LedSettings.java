//package org.firstinspires.ftc.teamcode.depricated.led;
//
//import android.graphics.Color;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartSettings;
//import org.firstinspires.ftc.teamcode.depricated.arm2.Arm2;
//
//public class LedSettings extends RobotPartSettings {
//
//    Supplier<Robot, Integer> led1Supplier = new Supplier<>((robot) -> (((Arm2) robot.getPartByClass(Arm2.class))).isBucketFullOrTimeout() ? Color.GREEN : Color.RED);
//    Supplier<Robot, Integer> led2Supplier = new Supplier<>(robot -> (led1Supplier.get()));
//    Supplier<Robot, Integer> led3Supplier = new Supplier<>(robot -> (led1Supplier.get()));
//    Supplier<Robot, Integer> led4Supplier = new Supplier<>(robot -> (led1Supplier.get()));
//    Supplier<Robot, Integer> led5Supplier = new Supplier<>(robot -> (led1Supplier.get()));
//    Supplier<Robot, Integer> led6Supplier = new Supplier<>(robot -> (led1Supplier.get()));
//    Supplier<Robot, Integer> led7Supplier = new Supplier<>(robot -> (led1Supplier.get()));
//    Supplier<Robot, Integer> led8Supplier = new Supplier<>(robot -> (led1Supplier.get()));
//    Supplier<Robot, Integer> led9Supplier = new Supplier<>(robot -> (led1Supplier.get()));
//    Supplier<Robot, Integer> led10Supplier = new Supplier<>(robot -> (led1Supplier.get()));
//
//    @Override
//    public void onInit(Robot robot) {
//        led1Supplier.setDevice(robot);
//        led2Supplier.setDevice(robot);
//        led3Supplier.setDevice(robot);
//        led4Supplier.setDevice(robot);
//        led5Supplier.setDevice(robot);
//        led6Supplier.setDevice(robot);
//        led7Supplier.setDevice(robot);
//        led8Supplier.setDevice(robot);
//        led9Supplier.setDevice(robot);
//        led10Supplier.setDevice(robot);
//    }
//}
