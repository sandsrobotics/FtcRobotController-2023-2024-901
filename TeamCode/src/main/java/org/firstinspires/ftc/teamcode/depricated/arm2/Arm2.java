//package org.firstinspires.ftc.teamcode.depricated.arm2;
//
//import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
//import org.firstinspires.ftc.teamcode.depricated.intake.Intake;
//import org.firstinspires.ftc.teamcode.depricated.led.Led;
//
//import om.self.ezftc.core.Robot;
//import om.self.ezftc.core.part.LoopedRobotPart;
//import om.self.ezftc.utils.Utils;
//import om.self.task.core.Group;
//import om.self.task.core.Task;
//import om.self.task.core.TaskEx;
//
//public class Arm2 extends LoopedRobotPart<Arm2Settings, Arm2Hardware> {
//	double bucketServoPos;
//	double armServoPos;
//	public int armMotorPos;
//	double capServoPos;
//	double keyServoPos;
//	double cheeseStartTime = 0;
//	int offset = 0;
//	double cheeseRange = 0;
//	int presetPos = 0;
//	int timesBucketFull = 0;
//
//	public Arm2(Robot robot) {
//		super(robot, "arm2", new Arm2Settings(), new Arm2Hardware());
//	}
//
//	public Arm2(Robot robot, Arm2Settings settings, Arm2Hardware hardware) {
//		super(robot, "arm2", settings, hardware);
//	}
//
//	@Override
//	public void onInit() {
//		armMotorPos = getSettings().armMotorStartPos;
//		armServoPos = getSettings().armServoStartPos;
//		bucketServoPos = getSettings().bucketServoStartPos;
//		capServoPos = getSettings().capServoStartPos;
//		keyServoPos = getSettings().keyServoStartPos;
//
//		getHardware().armMotor.setTargetPosition(getSettings().armMotorStartPos);
//		getHardware().armServo.setPosition(getSettings().armServoStartPos);
//		getHardware().bucketServo.setPosition(getSettings().bucketServoStartPos);
//		getHardware().capServo.setPosition(getSettings().capServoStartPos);
//		getHardware().keyServo.setPosition(getSettings().keyServoStartPos);
//
////		if (!((Intake) getParent().getPartByClass(Intake.class)).isAutonomous) {
////			getParent().taskManager.getMain().addBackgroundTask(makeAutoLiftBucketTask(), true);
////			getParent().taskManager.getMain().addBackgroundTask(makeLiftBucketTask(), false);
////		}
//	}
//
//	@Override
//	public void onRun() {
//		cheeseRange = getHardware().bucketRange.getDistance(DistanceUnit.INCH);
//		if (getParent().getPartByClass(Led.class) != null) {
//			if(cheeseRange < getSettings().blockSensorMinDist){
//				getParent().getPartByClass(Led.class).setLedStatus(1);
//			} else{
//				getParent().getPartByClass(Led.class).setLedStatus(0);
//			}
//		}
//
//		//armMotorPos = Utils.Math.capInt(armMotorPos + (int) (getSettings().armMotorMovementSupplier.getFloat() * getSettings().armMotorMovementSpeed), getSettings().armMotorMinPos, getSettings().armMotorMaxPos);
//		armMotorPos = Math.min(armMotorPos + (int) (getSettings().armMotorMovementSupplier.get() * getSettings().armMotorMovementSpeed), getSettings().armMotorMaxPos);
//		if (getHardware().limitSwitch.isPressed() && getSettings().armMotorMovementSupplier.get() != 0){
//			armMotorPos = Math.max(armMotorPos, 0);
//			if (armMotorPos > 25) armMotorPos = 0;
//			offset = getHardware().armMotor.getCurrentPosition();
//		}
//		//armServoPos = Utils.Math.capDouble(armServoPos + getSettings().armServoMovementSupplier.getInt() * getSettings().armServoMovementSpeed, getSettings().armServoMinPos, getSettings().armServoMaxPos);
//		//bucketServoPos = Utils.Math.capDouble(bucketServoPos + getSettings().bucketServoMovementSupplier.getInt() * getSettings().bucketServoMovementSpeed, getSettings().bucketServoMinPos, getSettings().bucketServoMaxPos);
//		capServoPos = Utils.Math.capDouble(capServoPos + getSettings().capServoMovementSupplier.get() * getSettings().capServoMovementSpeed, getSettings().capServoMinPos, getSettings().capServoMaxPos);
//		//keyServoPos = Utils.Math.capDouble(keyServoPos + getSettings().keyServoMovementSupplier.getInt() * getSettings().keyServoMovementSpeed, getSettings().keyServoMinPos, getSettings().keyServoMaxPos);
//
//		short armPreset = getSettings().armPresetSupplier.get();
//		armPreset--;
//		if (armPreset < 0) {
//			//setToAPresetPosition(preset);
//		} else {
//			presetPos = armPreset;
//				armServoPos = Utils.Math.capDouble(getSettings().armServoPresets[armPreset] + getSettings().armServoMovementSpeed, getSettings().armServoMinPos, getSettings().armServoMaxPos);
//				bucketServoPos = Utils.Math.capDouble(getSettings().bucketServoPresets[armPreset] + getSettings().bucketServoMovementSpeed, getSettings().bucketServoMinPos, getSettings().bucketServoMaxPos);
//				armMotorPos = Utils.Math.capInt(getSettings().armPresets[armPreset] + (int) (getSettings().armMotorMovementSupplier.get() * getSettings().armMotorMovementSpeed), getSettings().armMotorMinPos, getSettings().armMotorMaxPos);
//		}
//
//		short capPreset = (short) getSettings().capPresetSupplier.get();
//		capPreset--;
//		if (capPreset < 0) {
//			//setToAPresetPosition(preset);
//		} else {
//			capServoPos = Utils.Math.capDouble(getSettings().capServoPresets[capPreset] + getSettings().capServoMovementSpeed, getSettings().capServoMinPos, getSettings().capServoMaxPos);
//		}
//
//		short keyPreset = (short) getSettings().keyPresetSupplier.get();
//		keyPreset--;
//		if (keyPreset < 0) {
//			//setToAPresetPosition(preset);
//		} else {
//			keyServoPos = Utils.Math.capDouble(getSettings().keyServoPresets[keyPreset] + getSettings().keyServoMovementSpeed, getSettings().keyServoMinPos, getSettings().keyServoMaxPos);
//			getHardware().keyServo.setPosition(keyServoPos);
//		}
//		short dumpPreset = (short) getSettings().dumpPresetSupplier.get();
//		dumpPreset--;
//		if (dumpPreset < 0) {
//			//setToAPresetPosition(preset);
//		} else {
//			bucketServoPos = Utils.Math.capDouble(getSettings().dumpPresets[dumpPreset] + getSettings().bucketServoMovementSpeed, getSettings().bucketServoMinPos, getSettings().bucketServoMaxPos);
//			getHardware().bucketServo.setPosition(bucketServoPos);
//		}
//
//		getHardware().armMotor.setTargetPosition(armMotorPos + offset);
//		getHardware().armServo.setPosition(armServoPos);
//		getHardware().bucketServo.setPosition(bucketServoPos);
//		getHardware().capServo.setPosition(capServoPos);
//	}
//
//	public void autonomousPresets(short armPreset) {
//		armPreset--;
//		armServoPos = Utils.Math.capDouble(getSettings().armServoPresets[armPreset] + getSettings().armServoMovementSpeed, getSettings().armServoMinPos, getSettings().armServoMaxPos);
//		bucketServoPos = Utils.Math.capDouble(getSettings().bucketServoPresets[armPreset] + getSettings().bucketServoMovementSpeed, getSettings().bucketServoMinPos, getSettings().bucketServoMaxPos);
//		armMotorPos = Utils.Math.capInt(getSettings().armPresets[armPreset] + (int) (getSettings().armMotorMovementSupplier.get() * getSettings().armMotorMovementSpeed), getSettings().armMotorMinPos, getSettings().armMotorMaxPos);
//	}
//
//	public void autonomousDump(int preset) {
//		bucketServoPos = Utils.Math.capDouble(getSettings().dumpPresets[preset] + getSettings().bucketServoMovementSpeed, getSettings().bucketServoMinPos, getSettings().bucketServoMaxPos);
//	}
//
//	public void autonomousArmPreset() {
//		bucketServoPos = Utils.Math.capDouble(getSettings().dumpPresets[0] + getSettings().bucketServoMovementSpeed, getSettings().bucketServoMinPos, getSettings().bucketServoMaxPos);
//	}
//
//	public void armDown(int armPreset) {
//		presetPos = armPreset;
//		bucketServoPos = Utils.Math.capDouble(getSettings().bucketServoPresets[armPreset] + getSettings().bucketServoMovementSpeed, getSettings().bucketServoMinPos, getSettings().bucketServoMaxPos);
//		armServoPos = Utils.Math.capDouble(getSettings().armServoPresets[armPreset] + getSettings().armServoMovementSpeed, getSettings().armServoMinPos, getSettings().armServoMaxPos);
//		armMotorPos = Utils.Math.capInt(getSettings().armPresets[armPreset] + (int) (getSettings().armMotorMovementSupplier.get() * getSettings().armMotorMovementSpeed), getSettings().armMotorMinPos, getSettings().armMotorMaxPos);
//	}
//
//	public boolean isBucketFullOrTimeout() {
//		if (cheeseStartTime == 0) {
//			cheeseStartTime = System.currentTimeMillis();
//		}
//
//		double dist = cheeseRange; //if no worky worky this is why
//		if (dist < getSettings().blockSensorMinDist || System.currentTimeMillis() > cheeseStartTime + 2000) {//bucket full
//			cheeseStartTime = 0;
//			return true;
//		}
//		else return false;
//	}
//
//	public boolean isBucketFull(){
//		if(cheeseRange < getSettings().blockSensorMinDist){
//			timesBucketFull ++;
//			if(timesBucketFull >= 3)
//				return true;
//		}else{
//			timesBucketFull = 0;
//		}
//		return false;
//	}
//
//	@Override
//	public void onAddTelemetry() {
//		getParent().addTelemetry("arm motor", armMotorPos);
//		getParent().addTelemetry("arm servo", armServoPos);
//		getParent().addTelemetry("bucket servo", bucketServoPos);
//		getParent().addTelemetry("cap servo", capServoPos);
//		getParent().addTelemetry("key servo", keyServoPos);
//		getParent().addTelemetry("Cheese Range Inch", String.format("%.1f", cheeseRange));
//		getParent().addTelemetry("limit switch", getHardware().limitSwitch.isPressed());
//	}
//
//	@Override
//	public void onStop() {
//
//	}
//
//	private TaskEx makeAutoLiftBucketTask(){
//		TaskEx t = new TaskEx("auto lift bucket task");
//		t.addStep(() -> {
//			Task lift = (Task) getTaskManager().getChild("lift bucket task");
//			if(isBucketFull() && presetPos == 0 && !lift.isRunning())
//				lift.runCommand(Group.Command.START);
//		}, () -> (false));
//
//		return t;
//	}
//
//	private Task makeLiftBucketTask(){
//		Task t = new Task("lift bucket task");
//		Intake i = getParent().getPartByClass(Intake.class);
//
//		t.addStep(() -> {
//			i.pause(true);
//			i.startIntake(-0.8f);
//		});
//		t.addDelay(500);
//		t.addStep(() -> {
//			i.stopIntake();
//			i.unpause();
//			armServoPos = Utils.Math.capDouble(getSettings().armServoPresets[1] + getSettings().armServoMovementSpeed, getSettings().armServoMinPos, getSettings().armServoMaxPos);
//			bucketServoPos = Utils.Math.capDouble(getSettings().bucketServoPresets[1] + getSettings().bucketServoMovementSpeed, getSettings().bucketServoMinPos, getSettings().bucketServoMaxPos);
//			armMotorPos = Utils.Math.capInt(getSettings().armPresets[1] + (int) (getSettings().armMotorMovementSupplier.get() * getSettings().armMotorMovementSpeed), getSettings().armMotorMinPos, getSettings().armMotorMaxPos);
//			presetPos = 1;
//		});
//
//		return t;
//	}
//}
