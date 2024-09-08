package org.firstinspires.ftc.teamcode.parts.disklauncher;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.parts.disklauncher.hardware.DiskLauncherHardware;
import org.firstinspires.ftc.teamcode.parts.disklauncher.settings.DiskLauncherSettings;
import org.firstinspires.ftc.teamcode.parts.intake.IntakeControl;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.ezftc.core.part.LoopedPartImpl;

public class DiskLauncher extends ControllablePart<Robot, DiskLauncherSettings, DiskLauncherHardware, DiskLauncherControl> {
    public static boolean conveyerRunning;
    public static boolean launcherRunning;
    boolean conveyerWasPressed;
    boolean launcherWasPressed = false;
    public static boolean autoFlip = false;

    //***** Constructors *****
    public DiskLauncher(Robot parent) {
        super(parent, "DiskLauncher", () -> new DiskLauncherControl(false,false,false,false,false));
        setConfig(
                DiskLauncherSettings.makeDefault(),
                DiskLauncherHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }
    public DiskLauncher(Robot parent, DiskLauncherSettings settings, DiskLauncherHardware hardware, DiskLauncherControl control) {
        super(parent, "slider",() -> new DiskLauncherControl(false,false,false,false,false));
        setConfig(settings, hardware);
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {
        getHardware().launcherMotor.setPIDFCoefficients(DcMotorEx.RunMode.RUN_USING_ENCODER,getSettings().launcherMotorPID);
    }

    @Override
    public void onStart() {}

    @Override
    public void onRun(DiskLauncherControl diskLauncherControl) {
        if (launcherRunning) {
            setRPM(getSettings().targetWheelRpm);
            getHardware().doorServo.setPosition(0.335);
            //set color to Shot Red
            //getHardware().blinkinLeds.setPattern(RevBlinkinLedDriver.BlinkinPattern.SHOT_RED);
        } else {
            setRPM(0);
            getHardware().doorServo.setPosition(0.015);
            //when not running, set to rainbow rainbow palette (default)
            //getHardware().blinkinLeds.setPattern(getSettings().defaultLedPattern);
        }

        if (conveyerRunning) {
            getHardware().intakeMotor.setPower(1);
            //set color to beats per minute rainbow palette
            //getHardware().blinkinLeds.setPattern(RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_RAINBOW_PALETTE);
        } else {
            getHardware().intakeMotor.setPower(0);
        }
        parent.opMode.telemetry.addData("intakeMotor", getHardware().intakeMotor.getPower());
    }

    @Override
    public void onStop() {}

    public double getRPM() {
        return  getHardware().launcherMotor.getVelocity() * getSettings().spinMultiplier;
    }

    public void setRPM(double RPM) {
        getHardware().launcherMotor.setVelocity(RPM/getSettings().spinMultiplier);
    }
}
