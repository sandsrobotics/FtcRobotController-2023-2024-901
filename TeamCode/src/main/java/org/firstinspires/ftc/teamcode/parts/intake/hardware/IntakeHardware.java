package org.firstinspires.ftc.teamcode.parts.intake.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware {
    public static final double slideHoldPower = 1;
    public static final double sweepHoldPower = 1;
    public static final double robotLiftHoldPower = 1;

    public final DcMotor sliderMotor;
    public final DcMotor sweeperMotor;
    public final Servo sweepLiftServo;
    public final DcMotorEx robotLiftMotor;
    public final Servo grabberServo;
    public final DigitalChannel hangLiftLimitSwitch;
    public final Servo swingServoLeft;
    public final Servo swingServoRight;

    public IntakeHardware(DcMotor sliderMotor, DcMotor sweeperMotor, Servo sweepLiftServo, DcMotorEx robotLiftMotor, Servo grabberServo, DigitalChannel hangLiftLimitSwitch,Servo swingServoLeft,Servo swingServoRight) {
        this.sweeperMotor = sweeperMotor;
        this.sliderMotor = sliderMotor;
        this.sweepLiftServo = sweepLiftServo;
        this.robotLiftMotor = robotLiftMotor;
        this.grabberServo = grabberServo;
        this.hangLiftLimitSwitch = hangLiftLimitSwitch;
        this.swingServoLeft = swingServoLeft;
        this.swingServoRight = swingServoRight;
    }

    public static IntakeHardware makeDefault(HardwareMap hardwareMap) {
        MotorSettings slideMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_TO_POSITION, slideHoldPower);
        MotorSettings sweepMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, sweepHoldPower);
        ServoSettings sweepLiftServoSettings = new ServoSettings(ServoSettings.Number.ZERO_B, Servo.Direction.FORWARD);
        MotorSettings robotLiftMotorSettings = new MotorSettings(MotorSettings.Number.TWO_B, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_TO_POSITION, robotLiftHoldPower);
        ServoSettings grabberServoSettings = new ServoSettings(ServoSettings.Number.ZERO, Servo.Direction.FORWARD);
        ServoSettings swingServoLeftSettings = new ServoSettings(ServoSettings.Number.ONE, Servo.Direction.FORWARD);
        ServoSettings swingServoRightSettings = new ServoSettings(ServoSettings.Number.TWO, Servo.Direction.FORWARD);

        DigitalChannel limit = hardwareMap.get(DigitalChannel.class, "digital0");
        limit.setMode(DigitalChannel.Mode.INPUT);

        return new IntakeHardware(
                slideMotorSettings.makeMotor(hardwareMap),
                sweepMotorSettings.makeMotor(hardwareMap),
                sweepLiftServoSettings.makeServo(hardwareMap),
                robotLiftMotorSettings.makeExMotor(hardwareMap),
                grabberServoSettings.makeServo(hardwareMap),
                limit,
                swingServoLeftSettings.makeServo(hardwareMap),
                swingServoRightSettings.makeServo(hardwareMap)
        );
    }
}