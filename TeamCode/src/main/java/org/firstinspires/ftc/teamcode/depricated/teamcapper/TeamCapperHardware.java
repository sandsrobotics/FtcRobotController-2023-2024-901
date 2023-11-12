//package org.firstinspires.ftc.teamcode.depricated.teamcapper;
//
//import com.qualcomm.robotcore.hardware.Servo;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartHardware;
//import org.firstinspires.ftc.teamcode.other.hardware.servo.ServoSettings;
//
//public class TeamCapperHardware extends RobotPartHardware {
//
//    ServoSettings capperServoSettings = new ServoSettings(ServoSettings.Number.TWO, Servo.Direction.FORWARD, 0.5);
//
//    Servo capperServo;
//
//    @Override
//    public void onInit(Robot robot) {
//        capperServo = capperServoSettings.makeServo(robot.hardwareMap);
//    }
//}
