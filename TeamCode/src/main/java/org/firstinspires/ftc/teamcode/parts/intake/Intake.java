package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.depricated.lifter.Lifter;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.sql.Time;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.task.core.Group;
import om.self.task.core.TaskEx;
import om.self.task.other.TimedTask;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl>{
    private int slideTargetPosition;
    private int liftTargetPosition;
    private int swingTargetPosition;
    AprilTag tag;
    Drive drive;
    boolean isBottom;
    boolean isTop;
    double motorPower = 0;
    public boolean dropComplete = true;

    private final Group movementTask = new Group("auto movement",getTaskManager());

    private final TimedTask autoDropTask = new TimedTask(TaskNames.autoDrop, movementTask);
    private final TimedTask autoGrabTask = new TimedTask(TaskNames.autoGrab, getTaskManager());
    private final TimedTask autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
    private final TimedTask autoDockTask = new TimedTask(TaskNames.autoDock, getTaskManager());



    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0,0, 0,0, 0));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware){
        super(parent, "slider", () -> new IntakeControl(0, 0,0, 0,0, 0));
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

    private void setSwingPosition(int position){
        swingTargetPosition = position;
        switch (swingTargetPosition) {
            case 1:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftDropPosition);
                getHardware().swingServoRight.setPosition(getSettings().swingRightDropPosition);
                break;
            case 2:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftSafePosition);
                getHardware().swingServoRight.setPosition(getSettings().swingRightSafePosition);
                break;
            case 0:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftSafePosition);
                getHardware().swingServoRight.setPosition(getSettings().swingRightSafePosition);
                break;
        }
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

 /*   public void robotLiftWithPosition(int power, boolean force) {
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
            if (getHardware().robotLiftMotor.getCurrentPosition() >= getSettings().maxLiftPosition
                || getHardware().liftHighLimitSwitch.getState() == true)
                power = 0;
        }

        if (force)
            setRobotLiftPositionUnsafe(getHardware().robotLiftMotor.getCurrentPosition() + (int) power);
        else
            setRobotLiftPositionUnsafe(getHardware().robotLiftMotor.getCurrentPosition() + (int) power);
    }*/

    public void robotLiftWithPower(int power) {
        motorPower = power;

        if (power < 0) { // going down
            isTop = false;
            if (getHardware().liftLowLimitSwitch.getState() == true)
                motorPower = 0.0;
        } else { // going up
            if (getHardware().robotLiftMotor.getCurrent(CurrentUnit.MILLIAMPS) > 5000 || isTop) {
                isTop = true;
                motorPower = 0.0;
            }
        }
        getHardware().robotLiftMotor.setPower(motorPower);
    }

    public void constructAutoGrab(){
        autoGrabTask.autoStart = false;
        autoGrabTask.addStep(()->setSweepPosition(2));
        autoGrabTask.addStep(()->sweepWithPower(1));
    }

    public void constructAutoDock(){
        autoDockTask.autoStart = false;

        autoDockTask.addStep(()->setGrabPosition(1));
        autoDockTask.addStep(()->setSwingPosition(0));
        autoDockTask.addDelay(2000);
        autoDockTask.addStep(()->setSlidePosition(0));
    }

    public void startAutoDock() {autoDockTask.restart();}

    public void addAutoDockToTask(TimedTask task){
        task.addStep(autoDockTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.dockComplete));
    }

    public void constructAutoDrop(){
        autoDropTask.autoStart = false;

        autoDropTask.addStep(()->setSlidePosition(1500));
        autoDropTask.addDelay(5000);
        autoDropTask.addStep(()->setSwingPosition(1));
        autoDropTask.addStep(()->setGrabPosition(2));
    }

    public void startAutoDrop(){
        autoDropTask.restart();
    }

    public void addAutoDropToTask(TimedTask task){
        task.addStep(autoDropTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.dropComplete));
    }

    public void doTagRanging(DriveControl control) {
        boolean doTagRange = true;
        double xPower = 0.03;
        double yPower = 0.03;
        int tolerance = 1;

        if (tag.desiredTag != null && dropComplete) {

            tag.setDesiredTag(tag.desiredTag.id);
            boolean inLeft = tag.desiredTag.ftcPose.x < 1;
            boolean inRight = tag.desiredTag.ftcPose.x > -1;
            boolean inRange = tag.desiredTag.ftcPose.range <= 24.0;

           // if (inRange && getSlidePosition() > 1000) - use when i can actually use robot
            if (inRange) {
                if (inRight) // tag to the right (x is positive)
                    control.power = control.power.addX(xPower);
                else if (inLeft) //tag to the left (x is negative)
                    control.power = control.power.addX(-xPower);
            }
        }
    }

    @Override
    public void onInit() {
        constructAutoDrop();
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
        setSwingPosition(control.swingPosition);
        //robotLiftWithPosition(control.robotLiftPosition, false);
        robotLiftWithPower(control.robotLiftPosition);
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
        //drive.removeController(ContollerNames.distanceContoller);
    }

    public static final class ContollerNames {
        public static final String distanceContoller = "distance controller"; //TODO make better
    }

    public static final class TaskNames {
        public final static String autoGrab = "auto grab";
        public final static String autoDrop = "auto drop";
        public final static String autoHome = "auto home";
        public final static String autoDock = "auto dock";
    }

    public static final class Events {
        public static final String dockComplete = "DOCK_COMPLETE";
        public static final String grabComplete = "GRAB_COMPLETE";
        public static final String preDropComplete = "PRE_DROP_COMPLETE";
        public static final String dropComplete = "DROP_COMPlETE";
        public static  final String homeComplete = "HOME_COMPLETE";
    }
}

