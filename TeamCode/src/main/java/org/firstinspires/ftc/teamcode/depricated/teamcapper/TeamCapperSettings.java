//package org.firstinspires.ftc.teamcode.depricated.teamcapper;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartSettings;
//import org.firstinspires.ftc.teamcode.other.Utils;
//
//public class TeamCapperSettings extends RobotPartSettings {
//
//
//    ControlSupplier<Double> capperPresetSupplier = new ControlSupplier<>(gamepad -> (
//        gamepad.dpad_up ? 0.01 : gamepad.dpad_down ? -0.01 : 0
//        ), Utils.GamepadNum.ONE);
//    ControlSupplier<Short> capPresetSupplier = new ControlSupplier<>(gamepad -> ((short)((gamepad.dpad_left) ? 1 : (gamepad.dpad_right) ? 2 : 0)), Utils.GamepadNum.ONE);
//
//
//    // cap servo
//    double capServoMovementSpeed = .002;
//    //limits
//    double capServoMinPos = 0;
//    double capServoMaxPos = 1;
//    //servo presets           capPickup capDrop
//    double[] capServoPresets =   {0.29, 0.6};
//    //start
//    double capServoStartPos = 0;
//
//    //public double
//
//    @Override
//    public void onInit(Robot robot) {
//        capperPresetSupplier.init(robot);
//        capPresetSupplier.init(robot);
//    }
//}
