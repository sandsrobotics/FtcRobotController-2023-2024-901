package org.firstinspires.ftc.teamcode.parts.intake;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl>{
    private int slideTargetPosition;
    private int liftTargetPosition;
    boolean doTagRange = true;
    private int swingTargetPosition;
    private int slideSafePos = 1500; //TODO test this for real position
    private boolean reverse;
    AprilTag tag;
    Drive drive;
    boolean isTop;
    double motorPower = 0;
    public boolean dropComplete = true;
    private int pix = 0;
    private int pixLine = 0;
    private final int[] pixToPos = {1000,1300,1600,1900,2200}; //TODO move to settings, need 12 of these
    private final int[] pixLineToPos = {1000, 2000, 3000};
    private final Group movementTask = new Group("auto movement",getTaskManager());
    private final TimedTask autoDropTask = new TimedTask(TaskNames.autoDrop, movementTask);
    private final TimedTask autoGrabTask = new TimedTask(TaskNames.autoGrab, getTaskManager());
    private final TimedTask autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
    private final TimedTask autoDockTask = new TimedTask(TaskNames.autoDock, getTaskManager());

    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0,0, 0,0, 0, 0, 0, 0, 0));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware){
        super(parent, "slider", () -> new IntakeControl(0, 0,0, 0,0, 0, 0,0, 0, 0));
        setConfig(settings, hardware);
    }

    private void setSlideToHomeConfig(){
        double power = -0.125;

        getHardware().sliderMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //getHardware().rightLiftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        getHardware().sliderMotor.setPower(power);
        //getHardware().rightLiftMotor.setPower(power);
    }

    private void setMotorsToRunConfig(){
        getHardware().sliderMotor.setPower(IntakeHardware.slideHoldPower);
        //getHardware().rightLiftMotor.setPower(LifterHardware.liftHoldPower);
        getHardware().sliderMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //getHardware().rightLiftMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
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
        switch (position) {
            case 1:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftDropPosition);
                getHardware().swingServoRight.setPosition(getSettings().swingLeftDropPosition);
                break;
            case 2:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftSafePosition);
                getHardware().swingServoRight.setPosition(getSettings().swingLeftSafePosition);
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

    public double getTopPixelDist() { return getHardware().topSensor.getDistance(DistanceUnit.CM);}

    public double getBottomPixelDist() { return getHardware().botSensor.getDistance(DistanceUnit.CM);}

    public void setLaunchAngle(int position) {
        switch (position) {
            case 1:
                getHardware().launchServoAngle.setPosition(getSettings().launchAngleArmPosition);
                break;
            case 2:
                getHardware().launchServoAngle.setPosition(getSettings().launchAngleStorePosition);
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

    public int getPix(){return pix;}

    public void setPix(int pixel){
            pix = pix + pixel;
            if(pix > getSettings().maxPix)
                pix = getSettings().maxPix;
            else if(pix < 0)
                pix = 0;
        }

    public void robotLiftWithPower(int power) {
        motorPower = power;

        if (power < 0) { // going down
            isTop = false;
            if (getHardware().liftLowLimitSwitch.getState() == true)
                motorPower = 0.0;
        } else if(power > 0) { // going up
            if (getHardware().robotLiftMotor.getCurrent(CurrentUnit.MILLIAMPS) > 5000 || isTop) {
                isTop = true;
                motorPower = 0.0;
            }
        }
        getHardware().robotLiftMotor.setPower(motorPower);
    }

    public void constructAutoHome(){
        autoHomeTask.autoStart = false;

        autoHomeTask.addStep(()->setSwingPosition(2));

        autoHomeTask.addStep(()->setSlideToHomeConfig());

        autoHomeTask.addTimedStep(() -> {
            parent.opMode.telemetry.addData("homing", ")");
        }, () -> !getHardware().slideLowLimitSwitch.getState(), 5000);
        autoHomeTask.addStep(() -> {
            getHardware().sliderMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            //getHardware().rightLiftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

            getHardware().sliderMotor.setTargetPosition(0);
            //getHardware().rightLiftMotor.setTargetPosition(0);

            slideTargetPosition = 0;

            setMotorsToRunConfig();
        });
    }

    public void startAutoHome(){
        autoHomeTask.restart();
    }

    public void addAutoHomeToTask(TimedTask task){}

    public void constructAutoGrab(){
        autoGrabTask.autoStart = false;

        autoGrabTask.addTimedStep(()->sweepWithPower(reverse ? -.5 : 1), 5000);
        autoGrabTask.addStep(()->setGrabPosition(3));
    }

    public void startAutoGrab() {autoGrabTask.restart();}

    public void addAutoGrabToTask(TimedTask task, boolean rev){
        reverse = rev;
        task.addStep(autoGrabTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.grabComplete));
    }

    public void constructAutoDock(){
        autoDockTask.autoStart = false;

        autoDockTask.addStep(()->setGrabPosition(1));
        autoDockTask.addStep(()->setSwingPosition(2));
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
        autoDropTask.addStep(()->setSlidePosition(pixToPos[pix]));
        autoDropTask.addDelay(5000);
        if(getSlidePosition() > slideSafePos)
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

    public void doDummyRanging(DriveControl control) {}

    public void doTagRanging(DriveControl control) {
        double xPower = 0.03;
        double yPower = 0.03;
        int tolerance = 1;

        if (tag.desiredTag != null && doTagRange) {
            boolean inLeft = tag.desiredTag.ftcPose.x < 2;
            boolean inRight = tag.desiredTag.ftcPose.x > -2;
            boolean inRange = tag.desiredTag.ftcPose.range <= 15.0;
            boolean tooClose = tag.desiredTag.ftcPose.range <= 5.0;

           if (inRange && getSlidePosition() > 2500) {
               tag.setDesiredTag(tag.desiredTag.id); //lock the currrent tag
               if (inRight) // tag to the right (x is positive)
                    control.power = control.power.addX(xPower);
                else if (inLeft) //tag to the left (x is negative)
                    control.power = control.power.addX(-xPower);
            }
           else if (tooClose){
               drive.stopRobot();
               doTagRange = false;
           }
        }
    }

    @Override
    public void onInit() {
        constructAutoDrop();
        constructAutoDock();
        constructAutoGrab();
        constructAutoHome();
        //setSweepPosition(0);
        setSwingPosition(2);
        setLaunchAngle(2);
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) { //TODO separate keeping Slider motor position from onRun
        slideWithPower(control.sliderPower,false);
        sweepWithPower(control.sweeperPower);
        setGrabPosition(control.grabberPosition);
        robotLiftWithPower(control.robotLiftPosition);
        setLaunchAngle(control.launchPosition);
        setSwingPosition(control.swingPosition);

        parent.opMode.telemetry.addData("Top Pixel (cm)", getTopPixelDist());
        parent.opMode.telemetry.addData("Bottom Pixel (cm)", getBottomPixelDist());
        //parent.opMode.telemetry.addData("Sweep Speed", control.sweeperPower);
        //parent.opMode.telemetry.addData("Lift height", getRobotLiftPosition());
        //parent.opMode.telemetry.addData("dpad",control.robotLiftPosition);
        //parent.opMode.telemetry.addData("Lift Current", getHardware().robotLiftMotor.getCurrent(CurrentUnit.MILLIAMPS));
        //parent.opMode.telemetry.addData("Low Lift Switch", getHardware().liftLowLimitSwitch.getState() ? "closed" : "open");
        //parent.opMode.telemetry.addData("High Lift Switch", getHardware().liftHighLimitSwitch.getState() ? "closed" : "open");
    }

    @Override
    public void onSettingsUpdate(IntakeSettings IntakeSettings) {}

    @Override
    public void onHardwareUpdate(IntakeHardware IntakeHardware) {}

    @Override
    public void onStart() {
        drive = getBeanManager().getBestMatch(Drive.class, false);
        tag = getBeanManager().getBestMatch(AprilTag.class, false);
        drive.addController(Intake.ContollerNames.distanceContoller, (control) -> doTagRanging(control));
        //setSweepPosition(1);
    }

    @Override
    public void onStop() {
        drive.removeController(ContollerNames.distanceContoller);
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

