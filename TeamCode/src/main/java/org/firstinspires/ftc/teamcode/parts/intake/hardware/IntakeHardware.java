package org.firstinspires.ftc.teamcode.parts.intake.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class IntakeHardware {
    public static final double slideHoldPower = 1;
    public static final double sweepHoldPower = 1;

    public final DcMotor sliderMotor;
    public final DcMotor sweeperMotor;
    public final Servo sweepLiftServo;

    public IntakeHardware(DcMotor sliderMotor, DcMotor sweeperMotor, Servo sweepLiftServo) {
        this.sweeperMotor = sweeperMotor;
        this.sliderMotor = sliderMotor;
        this.sweepLiftServo = sweepLiftServo;
    }

    public static IntakeHardware makeDefault(HardwareMap hardwareMap) {
        MotorSettings slideMotorSettings = new MotorSettings(MotorSettings.Number.ZERO_B, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_TO_POSITION, slideHoldPower);
        MotorSettings sweepMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_WITHOUT_ENCODER, sweepHoldPower);
        ServoSettings sweepLiftServoSettings = new ServoSettings(ServoSettings.Number.ZERO_B, Servo.Direction.FORWARD);

        return new IntakeHardware(
                slideMotorSettings.makeMotor(hardwareMap),
                sweepMotorSettings.makeMotor(hardwareMap),
                sweepLiftServoSettings.makeServo(hardwareMap));
    }
}