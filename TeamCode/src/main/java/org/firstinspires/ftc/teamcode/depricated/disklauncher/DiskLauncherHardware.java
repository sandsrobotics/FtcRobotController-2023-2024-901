//package org.firstinspires.ftc.teamcode.depricated.disklauncher;
//
//import com.qualcomm.hardware.rev.RevTouchSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorEx;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartHardware;
//import org.firstinspires.ftc.teamcode.other.hardware.motor.MotorSettings;
//import org.firstinspires.ftc.teamcode.other.hardware.servo.ServoSettings;
//
//public class DiskLauncherHardware extends RobotPartHardware {
//    ////////////
//    //settings//
//    ////////////
//    public MotorSettings launcherMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 1.0);
//    public MotorSettings intakeMotorSettings = new MotorSettings(MotorSettings.Number.TWO_B, DcMotorSimple.Direction.REVERSE, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_USING_ENCODER, 1.0);
//
//    public ServoSettings flipServoSettings = new ServoSettings(ServoSettings.Number.ONE_B, Servo.Direction.REVERSE);
//    public ServoSettings doorServoSettings = new ServoSettings(ServoSettings.Number.FOUR_B, Servo.Direction.FORWARD);
//
//    ///////////
//    //objects//
//    ///////////
//    DcMotorEx launcherMotor;
//    DcMotorEx intakeMotor;
//    Servo flipServo;
//    Servo doorServo;
//
//    @Override
//    public void onInit(Robot robot) {
//        launcherMotor = launcherMotorSettings.makeExMotor(robot.hardwareMap);
//        intakeMotor = intakeMotorSettings.makeExMotor(robot.hardwareMap);
//        flipServo = flipServoSettings.makeServo(robot.hardwareMap);
//        doorServo = doorServoSettings.makeServo(robot.hardwareMap);
//    }
//}
