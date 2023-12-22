package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.depricated.lifter.Lifter;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.task.core.Group;
import om.self.task.core.TaskEx;
import om.self.task.other.TimedTask;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl>{
    private int slideTargetPosition;
    private int liftTargetPosition;
    AprilTag tag;
    Drive drive;

    private final Group movementTask = new Group("auto movement",getTaskManager());
    private final TimedTask doTagRanging = new TimedTask(TaskNames.doTagRanging, movementTask);

    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0,0, 0,0));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware){
        super(parent, "slider", () -> new IntakeControl(0, 0,0, 0,0));
        setConfig(settings, hardware);
    }

    public void slideWithPower(double power, boolean force){
        if(Math.abs(power) < getSettings().minRegisterVal) return;

        if(power < 0)
            power *= getSettings().maxDownSlideSpeed;
        else
            power *= getSettings().maxUpSlideSpeed;

        if(force)
            setSlidePositionUnsafe(getHardware().sliderMotor.getCurrentPosition() + (int)power);
        else
            setSlidePosition(getHardware().sliderMotor.getCurrentPosition() + (int)power);
    }

    public void sweepWithPower(double power) {
        getHardware().sweeperMotor.setPower(.42 * power);
    }

    public void setSlidePosition(int position){
        setSlidePositionUnsafe(Math.min(getSettings().maxSlidePosition, Math.max(getSettings().minSlidePosition, position)));
    }

    private void setSlidePositionUnsafe(int position){
        slideTargetPosition = position;
        getHardware().sliderMotor.setTargetPosition(position);
    }

    private void setRobotLiftPositionUnsafe(int position){
        liftTargetPosition = position;
        getHardware().robotLiftMotor.setTargetPosition(position);
    }

    public boolean isLiftInTolerance(){
        return Math.abs(slideTargetPosition - getSlidePosition()) <= getSettings().tolerance;
    }

    public void setLiftToTop(){
        setSlidePosition(getSettings().maxSlidePosition);
    }

    public void setLiftToBottom(){
        setSlidePosition(getSettings().minSlidePosition);
    }

    public int getSlidePosition(){
        return getHardware().sliderMotor.getCurrentPosition();
    }

    public int getRobotLiftPosition(){
        return getHardware().robotLiftMotor.getCurrentPosition();
    }

    public void setSweepPosition(int position) {
        switch (position) {
            case 1:
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoMinPosition);
                break;
            case 2:
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoMaxPosition);
                break;
        }
    }

    public void setGrabPosition(int position) {
        switch (position) {
            case 1:
                getHardware().grabberServo.setPosition(getSettings().grabberOpenPosition);
                break;
            case 2:
                getHardware().grabberServo.setPosition(getSettings().grabberGripOnePosition);
                break;
            case 3:
                getHardware().grabberServo.setPosition(getSettings().grabberGripTwoPosition);
                break;
        }
    }

    public void robotLiftWithPower(int power, boolean force) {
        if (Math.abs(power) < getSettings().minRegisterVal) return;

        if (power < 0) {
            power *= getSettings().maxDownLiftSpeed;
            if (getHardware().liftLowLimitSwitch.getState() == true) {
                if (getHardware().robotLiftMotor.getCurrentPosition() != 0) {
                    getHardware().robotLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    getHardware().robotLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    setRobotLiftPositionUnsafe(0);
                }
                power = 0;
            }
        } else {
            power *= getSettings().maxUpLiftSpeed;
            if (getHardware().robotLiftMotor.getCurrentPosition() >= getSettings().maxLiftPosition)
                power = 0;
        }

        if (force)
            setRobotLiftPositionUnsafe(getHardware().robotLiftMotor.getCurrentPosition() + (int) power);
        else
            setRobotLiftPositionUnsafe(getHardware().robotLiftMotor.getCurrentPosition() + (int) power);
    }

    public void doTagRanging(DriveControl control){
        if(tag.desiredTag != null){
            if (tag.desiredTag.ftcPose.x >= 5)
                control.power = control.power.addX(0.5);
            else if (tag.desiredTag.ftcPose.x <= 5)
                control.power = control.power.addX(-0.5);
        }
        else {
            //control.power = control.power.addY(-0.5);
        }
    }

    public void constructTagRanging(){

    }

    public void addDoTagRanging(TaskEx task){
        task.addStep(doTagRanging::restart);
    }

    public void startTagRanging(){
        doTagRanging.restart();
    }

    @Override
    public void onInit() {
        setSweepPosition(0);
    }
    
    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) { //TODO separate keeping Slider motor position from onRun
        slideWithPower(control.sliderPower,false);
        sweepWithPower(control.sweeperPower);
        setSweepPosition(control.sweepLiftPosition);
        setGrabPosition(control.grabberPosition);
        robotLiftWithPower(control.robotLiftPosition, false);
        parent.opMode.telemetry.addData("Slider height", getSlidePosition());
        parent.opMode.telemetry.addData("Sweep Speed", control.sweeperPower);
        parent.opMode.telemetry.addData("Lift height", getRobotLiftPosition());
        parent.opMode.telemetry.addData("dpad",control.robotLiftPosition);
        parent.opMode.telemetry.addData("Lift Current", getHardware().robotLiftMotor.getCurrent(CurrentUnit.MILLIAMPS));
        parent.opMode.telemetry.addData("Low Lift Switch", getHardware().liftLowLimitSwitch.getState() ? "closed" : "open");
        parent.opMode.telemetry.addData("High Lift Switch", getHardware().liftHighLimitSwitch.getState() ? "closed" : "open");
    }

    @Override
    public void onSettingsUpdate(IntakeSettings IntakeSettings) {}

    @Override
    public void onHardwareUpdate(IntakeHardware IntakeHardware) {

    }

    @Override
    public void onStart() {
        drive = getBeanManager().getBestMatch(Drive.class, false);
        tag = getBeanManager().getBestMatch(AprilTag.class, false);
        drive.addController(Intake.ContollerNames.distanceContoller, (control) -> doTagRanging(control));
        setSweepPosition(1);
    }

    @Override
    public void onStop() {
        drive.removeController(ContollerNames.distanceContoller);
    }

    public static final class ContollerNames {
        public static final String distanceContoller = "distance controller"; //TODO make better
    }

    public static final class TaskNames {
        public final static String doTagRanging = "do tag ranging";
    }
}

