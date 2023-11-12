package org.firstinspires.ftc.teamcode.parts.lifter2.hardware;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.lib.DFR304Range;

import om.self.ezftc.utils.hardware.motor.MotorSettings;
import om.self.ezftc.utils.hardware.servo.ServoSettings;

public class LifterHardware {
    public final DcMotor leftLiftMotor;

    public final Servo leftTurnServo;
    public final Servo rightTurnServo;
    public final AnalogInput turnSensor;

    public final Servo grabServo;

    public final DFR304Range leftUltrasonic;
    public final DFR304Range rightUltrasonic;
    public final DFR304Range midUltrasonic;

    public final DigitalChannel limitSwitch;
    public final RevColorSensorV3 coneSensor;

    public LifterHardware(DcMotor leftLiftMotor, Servo leftTurnServo, Servo rightTurnServo, AnalogInput turnSensor, Servo grabServo, DFR304Range leftUltrasonic, DFR304Range rightUltrasonic, DFR304Range midUltrasonic, DigitalChannel limitSwitch, RevColorSensorV3 coneSensor) {
        this.leftLiftMotor = leftLiftMotor;
        this.leftTurnServo = leftTurnServo;
        this.rightTurnServo = rightTurnServo;
        this.turnSensor = turnSensor;
        this.grabServo = grabServo;
        this.leftUltrasonic = leftUltrasonic;
        this.rightUltrasonic = rightUltrasonic;
        this.midUltrasonic = midUltrasonic;
        this.limitSwitch = limitSwitch;
        this.coneSensor = coneSensor;
    }

    public static LifterHardware makeDefault(HardwareMap hardwareMap){
        return new LifterHardware(
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null,
                null
        );
    }
}
