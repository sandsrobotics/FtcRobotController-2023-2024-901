//package org.firstinspires.ftc.teamcode.depricated.arm2;
//
//import com.qualcomm.hardware.rev.RevColorSensorV3;
//import com.qualcomm.hardware.rev.RevTouchSensor;
//import com.qualcomm.robotcore.hardware.ColorSensor;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.DigitalChannel;
//import com.qualcomm.robotcore.hardware.DistanceSensor;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartHardware;
//import org.firstinspires.ftc.teamcode.other.hardware.motor.MotorSettings;
//import org.firstinspires.ftc.teamcode.other.hardware.servo.ServoSettings;
////TODO finish settings for hardware
//public class Arm2Hardware extends RobotPartHardware {
//	////////////
//	//settings//
//	////////////
//	public MotorSettings armMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_TO_POSITION, 0.6);
//	public ServoSettings armServoSettings = new ServoSettings(ServoSettings.Number.ONE, Servo.Direction.REVERSE);
//	public ServoSettings bucketServoSettings = new ServoSettings(ServoSettings.Number.THREE, Servo.Direction.REVERSE);
//	public ServoSettings capServoSettings = new ServoSettings(ServoSettings.Number.FIVE, Servo.Direction.REVERSE);
//	public ServoSettings keyServoSettings = new ServoSettings(ServoSettings.Number.FOUR, Servo.Direction.REVERSE);
//
//	///////////
//	//objects//
//	///////////
//	DcMotor armMotor;
//	Servo armServo;
//	Servo bucketServo;
//	Servo capServo;
//	Servo keyServo;
//	DistanceSensor bucketRange;
//	RevTouchSensor limitSwitch;
//
//	@Override
//	public void onInit(Robot robot) {
//		armMotor = armMotorSettings.makeMotor(robot.hardwareMap);
//		armServo = armServoSettings.makeServo(robot.hardwareMap);
//		bucketServo = bucketServoSettings.makeServo(robot.hardwareMap);
//		capServo = capServoSettings.makeServo(robot.hardwareMap);
//		keyServo = keyServoSettings.makeServo(robot.hardwareMap);
//		bucketRange = robot.hardwareMap.get(DistanceSensor.class, "range1");
//		limitSwitch = robot.hardwareMap.get(RevTouchSensor.class, "limit");
//	}
//}
