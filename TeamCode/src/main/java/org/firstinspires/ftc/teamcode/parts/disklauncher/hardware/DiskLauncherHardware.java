package org.firstinspires.ftc.teamcode.parts.disklauncher.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class DiskLauncherHardware {
    public final DcMotorEx launcherMotor;
    public final DcMotorEx intakeMotor;
    public final Servo flipServo;
    public final Servo doorServo;
    public final RevBlinkinLedDriver blinkinLeds;

    public DiskLauncherHardware(DcMotorEx launcherMotor, DcMotorEx intakeMotor, Servo flipServo, Servo doorServo, RevBlinkinLedDriver blinkinLeds) {
        this.launcherMotor = launcherMotor;
        this.intakeMotor = intakeMotor;
        this.flipServo = flipServo;
        this.doorServo = doorServo;
        this.blinkinLeds = blinkinLeds;
    }
    public static DiskLauncherHardware makeDefault(HardwareMap hardwareMap) {
        MotorSettings launcherMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 1.0);
        MotorSettings intakeMotorSettings = new MotorSettings(MotorSettings.Number.TWO_B, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 1.0);
        ServoSettings flipServoSettings = new ServoSettings(ServoSettings.Number.ONE_B, Servo.Direction.REVERSE);
        ServoSettings doorServoSettings = new ServoSettings(ServoSettings.Number.FOUR_B, Servo.Direction.FORWARD);
        return new DiskLauncherHardware(
            launcherMotorSettings.makeExMotor(hardwareMap),
            intakeMotorSettings.makeExMotor(hardwareMap),
            flipServoSettings.makeServo(hardwareMap),
            doorServoSettings.makeServo(hardwareMap),
            hardwareMap.get(RevBlinkinLedDriver.class, "servo5B")
        );
    }
}
