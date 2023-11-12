//package org.firstinspires.ftc.teamcode.depricated.armtest;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartSettings;
//import org.firstinspires.ftc.teamcode.other.Utils;
//
//public class ArmTestSettings extends RobotPartSettings {
//    ////////////
//    //settings//
//    ////////////
//    //input
//    Utils.GamepadNum gamepadNum = Utils.GamepadNum.ONE;
//    //arm
//    ControlSupplier<Float> armMovementSupplier = new ControlSupplier<>(gamepad -> (gamepad.right_trigger - gamepad.left_trigger), Utils.GamepadNum.ONE);
//    float minInputRegisterVal = 0.1f;
//
//    //ControlSupplier<Float> ejectServoSupplier = new ControlSupplier<Float>(gamepad -> gamepad.a ? 0.7f : 0.4f, Utils.GamepadNum.ONE);
//
//    ControlSupplier<Integer> ejectWheelSupplier = new ControlSupplier<Integer>(gamepad -> ((gamepad.x ? -1 : 0) + (gamepad.y ? 1 : 0)), Utils.GamepadNum.ONE);
//
//
//    //arm
//    //speed
//    float armMovementSpeed = 1f;//0.7f;
//    //limits
//    int armMinPos = 0;
//    int armMaxPos = 3000;
//    //start
//    int armStartPos = 0;
//
//
//    @Override
//    public void onInit(Robot robot) {
//        armMovementSupplier.init(robot);
//        //ejectServoSupplier.init(robot);
//        ejectWheelSupplier.init(robot);
//    }
//}
