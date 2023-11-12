//package org.firstinspires.ftc.teamcode.depricated.armtest;
//
//import com.qualcomm.robotcore.hardware.CRServo;
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartHardware;
//import org.firstinspires.ftc.teamcode.other.hardware.motor.MotorSettings;
//import org.firstinspires.ftc.teamcode.other.hardware.servo.ServoSettings;
//
//public class ArmTestHardware extends RobotPartHardware {
//    ////////////
//    //settings//
//    ////////////
//    public MotorSettings armMotorSettings = new MotorSettings(MotorSettings.Number.ONE_B, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE, DcMotor.RunMode.RUN_TO_POSITION, 0.1);
//
//    //public ServoSettings ejectServoSettings = new ServoSettings(ServoSettings.Number.FIVE_B);
//
//    ///////////
//    //objects//
//    ///////////
//    DcMotor armMotor;
//    //Servo ejectServo;
//    CRServo wheelServo;
//    CRServo wheelServo2;
//
//    @Override
//    public void onInit(Robot robot) {
//        armMotor = armMotorSettings.makeMotor(robot.hardwareMap);
//        //ejectServo = ejectServoSettings.makeServo(robot.hardwareMap);
//        wheelServo = robot.hardwareMap.get(CRServo.class, ServoSettings.Number.ONE_B.value);
//        wheelServo2 = robot.hardwareMap.get(CRServo.class, ServoSettings.Number.TWO_B.value);
//    }
//}
