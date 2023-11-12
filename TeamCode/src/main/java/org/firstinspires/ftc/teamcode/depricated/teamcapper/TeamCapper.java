//package org.firstinspires.ftc.teamcode.depricated.teamcapper;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.LoopedRobotPart;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartHardware;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartSettings;
//import org.firstinspires.ftc.teamcode.other.Utils;
//
//public class TeamCapper extends LoopedRobotPart<TeamCapperHardware, TeamCapperSettings> {
//    double capServoPos;
//
//    public TeamCapper(Robot robot, TeamCapperHardware hardware, TeamCapperSettings settings) {
//        super(robot, hardware, settings);
//    }
//
//    public TeamCapper(Robot robot){
//        super(robot, new TeamCapperHardware(), new TeamCapperSettings());
//    }
//
//    @Override
//    public void onConstruct() {
//
//    }
//
//    @Override
//    public void onInit() {
//        capServoPos = settings.capServoStartPos;
//        hardware.capperServo.setPosition(settings.capServoStartPos);
//    }
//
//    @Override
//    public void onStart() {
//
//    }
//
//    @Override
//    public void onPause() {
//
//    }
//
//    @Override
//    public void onUnpause() {
//
//    }
//
//    @Override
//    public void onRunLoop(short runMode) {
//        if(runMode == 1){
//            capServoPos = (hardware.capperServo.getPosition() + settings.capperPresetSupplier.get());
//
//            short capPreset = (short) settings.capPresetSupplier.get();
//            capPreset--;
//            if (capPreset < 0) {
//                //setToAPresetPosition(preset);
//            } else {
//                capServoPos = Utils.Math.capDouble(settings.capServoPresets[capPreset] + settings.capServoMovementSpeed, settings.capServoMinPos, settings.capServoMaxPos);
//            }
//            hardware.capperServo.setPosition(capServoPos);
//        }
//    }
//
//    @Override
//    public void onAddTelemetry() {
//        robot.addTelemetry("capper pos", hardware.capperServo.getPosition());
//    }
//
//    @Override
//    public void onStop() {
//
//    }
//}
