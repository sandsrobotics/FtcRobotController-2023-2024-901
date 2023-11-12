//package org.firstinspires.ftc.teamcode.depricated.disklauncher;
//
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.LoopedRobotPart;
//
//public class DiskLauncher extends LoopedRobotPart<DiskLauncherHardware, DiskLauncherSettings> {
//    boolean conveyerRunning;
//    boolean launcherRunning;
//    boolean conveyerWasPressed;
//    boolean launcherWasPressed = false;
//
//    public DiskLauncher(Robot robot) {
//        super(robot, new DiskLauncherHardware(), new DiskLauncherSettings());
//    }
//
//    public DiskLauncher(Robot robot, DiskLauncherHardware hardware, DiskLauncherSettings settings) {
//        super(robot, hardware, settings);
//    }
//
//    @Override
//    public void onConstruct() {
//
//    }
//
//    @Override
//    public void onInit() {
//        hardware.launcherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,settings.launcherMotorPID);
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
//        if ( settings.launcherPresetSupplier.get()) {
//            launcherWasPressed = true;
//        } else if(launcherWasPressed) {
//            launcherRunning = !launcherRunning;
//            launcherWasPressed = false;
//        }
//
//        if (launcherRunning) {
//            setRPM(settings.targetWheelRpm);
//            hardware.doorServo.setPosition(0.3);
//        } else {
//            setRPM(0);
//            hardware.doorServo.setPosition(0);
//        }
//
//        if((settings.flipPresetSupplierSafety.get() || settings.flipPresetSupplier.get()) && settings.flipPresetSupplierSafetyOverride.get()) {
//            hardware.flipServo.setPosition(0);
//        }else {
//            hardware.flipServo.setPosition(1);
//        }
//
//        if (settings.conveyerPresetSupplier.get()) {
//            conveyerWasPressed = true;
//        } else if(conveyerWasPressed) {
//            conveyerRunning = !conveyerRunning;
//            conveyerWasPressed = false;
//        }
//
//        if (conveyerRunning) {
//            hardware.intakeMotor.setTargetPower(1);
//        } else {
//            hardware.intakeMotor.setTargetPower(0);
//        }
//    }
//
//    @Override
//    public void onAddTelemetry() {
//        robot.addTelemetry("spinner RPM", getRPM());
//    }
//
//    @Override
//    public void onStop() {
//
//    }
//
//    public double getRPM() {
//        return  hardware.launcherMotor.getVelocity() * settings.spinMultiplier;
//    }
//
//    public void setRPM(double RPM) {
//        hardware.launcherMotor.setVelocity(RPM/settings.spinMultiplier);
//    }
//}
