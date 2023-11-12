//package org.firstinspires.ftc.teamcode.depricated.duckspinner;
//
//import com.qualcomm.robotcore.hardware.DcMotor;
//import com.qualcomm.robotcore.hardware.DcMotorSimple;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.RobotPartHardware;
//import org.firstinspires.ftc.teamcode.other.hardware.motor.MotorSettings;
//
//public class DuckSpinnerHardware extends RobotPartHardware {
//	public MotorSettings duckSpinnerMotorSettings = new MotorSettings(MotorSettings.Number.THREE_B, DcMotorSimple.Direction.FORWARD, DcMotor.ZeroPowerBehavior.BRAKE);
//
//	DcMotor duckSpinnerMotor;
//
//	@Override
//	public void onInit(Robot robot){
//		duckSpinnerMotor = duckSpinnerMotorSettings.makeMotor(robot.hardwareMap);
//	}
//}