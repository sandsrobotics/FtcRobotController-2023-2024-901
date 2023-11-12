//package org.firstinspires.ftc.teamcode.depricated.disklauncher;
//
//import com.qualcomm.robotcore.hardware.PIDFCoefficients;
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartSettings;
//import org.firstinspires.ftc.teamcode.other.Utils;
//
//public class DiskLauncherSettings extends RobotPartSettings {
//    ControlSupplier<Boolean> flipPresetSupplier = new ControlSupplier<>(gamepad -> (gamepad.right_trigger > 0.1), Utils.GamepadNum.ONE);
//    ControlSupplier<Boolean> flipPresetSupplierSafety = new ControlSupplier<>(gamepad -> (gamepad.right_trigger > 0.7), Utils.GamepadNum.TWO);
//    ControlSupplier<Boolean> flipPresetSupplierSafetyOverride = new ControlSupplier<>(gamepad -> gamepad.left_bumper, Utils.GamepadNum.ONE);
//    DoubleControlSupplier launcherPresetSupplier = new DoubleControlSupplier(gamepad -> (gamepad.b));
//    DoubleControlSupplier conveyerPresetSupplier = new DoubleControlSupplier(gamepad -> (gamepad.a));
//
//    PIDFCoefficients launcherMotorPID = new PIDFCoefficients(100,0,0,12.4);
//
//    double ticksPerRev = 28;
//    double gearRatio = 1;
//    //double startRPM=1000;
//    double spinMultiplier = 60 / ticksPerRev * gearRatio;
//    double targetWheelRpm = 3700;
//
//    @Override
//    public void onInit(Robot robot) {
//        flipPresetSupplier.init(robot);
//        flipPresetSupplierSafety.init(robot);
//        flipPresetSupplierSafetyOverride.init(robot);
//        launcherPresetSupplier.init(robot);
//        conveyerPresetSupplier.init(robot);
//    }
//}
