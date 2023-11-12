//package org.firstinspires.ftc.teamcode.depricated.duckspinner;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.LoopedRobotPart;
//
//public class DuckSpinner extends LoopedRobotPart<DuckSpinnerHardware, DuckSpinnerSettings> {
//
//	public DuckSpinner(Robot robot, DuckSpinnerHardware hardware, DuckSpinnerSettings settings) {
//		super(robot, hardware, settings);
//	}
//
//	public DuckSpinner(Robot robot){
//		super(robot, new DuckSpinnerHardware(), new DuckSpinnerSettings());
//	}
//
//	/////////////////////
//	//LoopedRobotPart Methods//
//	/////////////////////
//	@Override
//	public void onConstruct() {
//
//	}
//
//	@Override
//	public void onInit() {
//
//	}
//
//	@Override
//	public void onStart() {
//
//	}
//
//	@Override
//	public void onPause() {
//
//	}
//
//	@Override
//	public void onUnpause() {
//
//	}
//
//	@Override
//	public void onRunLoop(short runMode) {
//		if(runMode == 1){
//			hardware.duckSpinnerMotor.setTargetPower(settings.duckSpinnerPowerSupplier.getRampedFloat());
//		}
//		else if(runMode == 2){
//			hardware.duckSpinnerMotor.setTargetPower(robot.autoBlue ? 0.5 : -0.5);
//		}
//	}
//
//	@Override
//	public void onAddTelemetry() {
//		robot.addTelemetry("duck spinner power", hardware.duckSpinnerMotor.getPower());
//	}
//
//	@Override
//	public void onStop() {
//
//	}
//}