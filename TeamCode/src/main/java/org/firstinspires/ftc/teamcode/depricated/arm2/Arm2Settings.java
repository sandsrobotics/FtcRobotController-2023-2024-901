//package org.firstinspires.ftc.teamcode.depricated.arm2;
//
//import com.qualcomm.robotcore.hardware.Gamepad;
//
//import om.self.ezftc.core.Robot;
//import om.self.supplier.suppliers.LinkedSupplier;
//
////TODO finish the settings for arm2
//public class Arm2Settings extends RobotPartSettings {
//	////////////
//	//settings//
//	////////////
//	//arm motor
//	LinkedSupplier<Gamepad, Float> armMotorMovementSupplier = new LinkedSupplier<>(gamepad -> (gamepad.right_trigger - gamepad.left_trigger));
//	float minInputRegisterVal = 0.1f;
//	//arm servo
//	//InputSupplier armServoMovementSupplier = new InputSupplier(gamepad -> ((gamepad.x) ? 1 : (gamepad.b) ? -1 : 0), Utils.GamepadNum.TWO);
//	//bucket servo
//	//InputSupplier bucketServoMovementSupplier = new InputSupplier(gamepad -> ((gamepad.dpad_left) ? 1 : (gamepad.dpad_right) ? -1 : 0), Utils.GamepadNum.TWO);
//	//cap servo
//	ControlSupplier<Integer> capServoMovementSupplier = new ControlSupplier<>(gamepad -> ((gamepad.dpad_down) ? 1 : (gamepad.dpad_up) ? -1 : 0), Utils.GamepadNum.ONE);
//	//key servo
//	//InputSupplier keyServoMovementSupplier = new InputSupplier(gamepad -> ((gamepad.a) ? 1 : (gamepad.b) ? -1 : 0), Utils.GamepadNum.TWO);
//	//preset
//	ControlSupplier<Short> armPresetSupplier = new ControlSupplier<>(gamepad -> ((short)((gamepad.dpad_down) ? 1 : (gamepad.dpad_left) ? 2 : (gamepad.dpad_right) ? 3 : (gamepad.dpad_up) ? 4 : (gamepad.y) ? 5 : 0)), Utils.GamepadNum.TWO);
//
//	ControlSupplier<Short> dumpPresetSupplier = new ControlSupplier<>(gamepad -> ((short)((gamepad.y) ? 1 : (gamepad.x) ? 2 : 0)), Utils.GamepadNum.ONE);
//
//	ControlSupplier<Short> capPresetSupplier = new ControlSupplier<>(gamepad -> ((short)((gamepad.dpad_left) ? 1 : (gamepad.dpad_right) ? 2 : 0)), Utils.GamepadNum.ONE);
//
//	ControlSupplier<Short> keyPresetSupplier = new ControlSupplier<>(gamepad -> ((short)((gamepad.left_bumper) ? 1 : (gamepad.right_bumper) ? 2 : 0)), Utils.GamepadNum.TWO);
//
//
//
//	//bucket servo
//	//speed
//	int bucketServoSpeed = 300; //in degrees/second
//	double bucketServoMovementSpeed = .03;
//	//limits
//	double bucketServoMinPos = 0;
//	double bucketServoMaxPos = 1;
//	//servo presets             	pickup	lift	bottom	middle  top
//	double[] bucketServoPresets =   {0.58,	0.87,   0.14,   0.14,   0.23};
//	double[] dumpPresets = {0.425,0.35};
//	//double[] bucketServoPresets =   {0.57,	0.76,   0.33,   0.32,   0.425,  0.425};
//	//start
//	double bucketServoStartPos = bucketServoPresets[1];
//
//	//arm servo
//	//speed
//	int armServoSpeed = 300; //in degrees/second
//	double armServoMovementSpeed = .005;
//	//limits
//	double armServoMinPos = 0;
//	double armServoMaxPos = 1;
//	//servo presets             pickup	lift  bottom  middle	top
//	double[] armServoPresets =   {0, 0.05, 0.995, 0.995, 0.905};
//	//start
//	double armServoStartPos = armServoPresets[1];
//
//	//arm motor
//	//speed
//	int armMotorMovementSpeed = 6;
//	//limits
//	int armMotorMinPos = 0;
//	int armMotorMaxPos = 3617;
//	//arm presets    pickup lift    bottom   middle top
//	int[] armPresets = {0,     0,     0,      475,  1130};
//	//550 old mid pos
//	//other
//	int armTolerance = 60;
//	//start
//	int armMotorStartPos = armPresets[1];
//
//    // cap servo
//	double capServoMovementSpeed = .002;
//	//limits
//	double capServoMinPos = 0;
//	double capServoMaxPos = 1;
//	//servo presets             capStart capPickup
//	double[] capServoPresets =   {0.16, 0.835};
//	//start
//	double capServoStartPos = capServoPresets[0];
//
//	//key servo
//	//speed
//	int keyServoSpeed = 300; //in degrees/second
//	double keyServoMovementSpeed = .005;
//	//limits
//	double keyServoMinPos = 0;
//	double keyServoMaxPos = 1;
//	//servo presets             keyOpen keyClose
//	double[] keyServoPresets =   {0.7,   0.315};
//	//start
//	double keyServoStartPos = keyServoPresets[1];
//
//	//other
//	double blockSensorMinDist = 3;
//
//
//	public Arm2Settings(Robot robot) {
//		armMotorMovementSupplier.setInput(robot.opMode.gamepad1);
//		//armServoMovementSupplier.init(robot);
//		//bucketServoMovementSupplier.init(robot);
//		capServoMovementSupplier.init(robot);
//		//keyServoMovementSupplier.init(robot);
//		armPresetSupplier.init(robot);
//		dumpPresetSupplier.init(robot);
//		capPresetSupplier.init(robot);
//		keyPresetSupplier.init(robot);
//	}
//}
//;