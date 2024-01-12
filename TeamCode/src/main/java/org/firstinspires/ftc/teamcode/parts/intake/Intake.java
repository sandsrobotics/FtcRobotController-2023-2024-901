package org.firstinspires.ftc.teamcode.parts.intake;

import android.graphics.Color;

import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.depricated.lifter.Lifter;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;
import org.firstinspires.ftc.teamcode.parts.led.Led;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

import static java.lang.Boolean.FALSE;
import static java.lang.Boolean.TRUE;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl>{
    private int slideTargetPosition;
    private int liftTargetPosition;
    public boolean doTagRange = true;
    public boolean doTagCenter = false;
    private boolean armed;
    private int swingTargetPosition;
    private final int slideSafePos = 970;
    private int ranging = 1;
    private int launchState;
    boolean atTag;
    boolean tooClose;
    boolean inRange;
    double currentDist;
    boolean inCenter;
    private boolean reverse;
    AprilTag tag;
    Drive drive;
    Led led;
    boolean isTop;
    double motorPower = 0;
    public boolean dropComplete = true;
    private int pix = 0;
    int grabTime;
    boolean botPixel, topPixel = false;
    private int pixLine = 0;
    private final int[] pixToPos = {1000,1100,1360,1600,1860}; //TODO move to settings, need 12 of these
    private final int[] pixLineToPos = {1000, 2000, 3000};
    private final Group movementTask = new Group("auto movement",getTaskManager());
    private final TimedTask autoDropTask = new TimedTask(TaskNames.autoDrop, movementTask);
    private final TimedTask autoGrabTask = new TimedTask(TaskNames.autoGrab, getTaskManager());
    private final TimedTask autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
    private final TimedTask autoDockTask = new TimedTask(TaskNames.autoDock, getTaskManager());
    private final TimedTask autoArmTask = new TimedTask(TaskNames.autoArm, getTaskManager());
    private final TimedTask autoStoreTask = new TimedTask(TaskNames.autoStore, getTaskManager());
    private  final TimedTask finishDropTask = new TimedTask(TaskNames.autoFinishDrop, movementTask);


    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0,0, 0,0, 0, 0, 0, 0,1));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware){
        super(parent, "slider", () -> new IntakeControl(0, 0,0, 0,0, 0, 0,0, 0,0));
        setConfig(settings, hardware);
    }

    private void preAutoMove(){
        triggerEvent(ControllablePart.Events.stopControllers);
    }
    private void postAutoMove(){
        triggerEvent(ControllablePart.Events.startControllers);
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

//    public double getBottomPixelColor() { return getHardware().botSensor.
////        // hsvValues is an array that will hold the hue, saturation, and value information.
////        float hsvValues[] = {0F, 0F, 0F};
////        // values is a reference to the hsvValues array.
////        final float values[] = hsvValues;
////        // sometimes it helps to multiply the raw RGB values with a scale factor
////        // to amplify/attentuate the measured values.
////        final double SCALE_FACTOR = 255;
////            Color.RGBToHSV((int) (robot.sensorColor.red() * SCALE_FACTOR),
////                    (int) (robot.sensorColor.green() * SCALE_FACTOR),
////                    (int) (robot.sensorColor.blue() * SCALE_FACTOR),
////                    hsvValues);
//                    return Color.RGBToHSV((int) (getHardware().botSensor.red() * 255),
//                    (int) (getHardware().botSensor.green() * 255),
//                    (int) (getHardware().botSensor.blue() * 255),
//                    hsvValues);
//    }
//

    public int hasPixels(){
        if (getTopPixelDist() < 2 && getBottomPixelDist() < 2)
            return 2;
        else if (getTopPixelDist() > 2 && getBottomPixelDist() < 2)
            return 1;
        else
            return 0;
    }

    public void setLeds(int pixels) {
        if (pixels == 1) {
            led.setBottomGroup(true);
            led.setTopGroup(false);
        } else if(pixels == 2) {
            led.setTopGroup(true);
            led.setBottomGroup(true);
        } else {
            led.setBottomGroup(false);
            led.setTopGroup(false);
        }
    }

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

    public void setLaunchState(int state){
        switch (state){
            case 1:
                if(armed)
                    getHardware().launchServoRelease.setPosition(getSettings().launchReleaseUnlockPosition);
                break;
            case 0:
                getHardware().launchServoRelease.setPosition(getSettings().launchReleaseLockPosition);
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
            case 4:
                getHardware().grabberServo.setPosition(getSettings().grabberPrimePosition);
                break;
        }
    }

    public int getPix(){return pix;}

    public void setPix(int pix) {
        if(pix > getSettings().maxPix) pix = getSettings().maxPix;
        else if(pix < 0) pix = 0;
        this.pix = pix;
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

        autoHomeTask.addStep(()->setGrabPosition(1));
        autoHomeTask.addStep(()->setSwingPosition(2));

        autoHomeTask.addStep(()->setSlideToHomeConfig());

        autoHomeTask.addTimedStep(() -> {parent.opMode.telemetry.addData("homing", getHardware().slideLowLimitSwitch.getState());}, () -> !getHardware().slideLowLimitSwitch.getState(), 10000);
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

    public void constructAutoArm(){
        autoArmTask.autoStart = false;

        autoArmTask.addStep(this::preAutoMove);
        autoArmTask.addTimedStep(()->robotLiftWithPower(1), 2000);;
        autoArmTask.addStep(()->setLaunchAngle(1));
        autoArmTask.addStep(this::postAutoMove);
        autoArmTask.addStep(()->triggerEvent(Events.armComplete));
    }

    public void startAutoArm() {
        armed = true;
        autoArmTask.restart();
    }

    public void constructAutoStore(){
        autoStoreTask.autoStart = false;

        autoStoreTask.addStep(this::preAutoMove);
        autoStoreTask.addStep(()->setLaunchAngle(2));
        autoStoreTask.addStep(this::postAutoMove);
        autoStoreTask.addStep(()->triggerEvent(Events.storeComplete));
    }

    public void startAutoStore(){
        armed = false;
        autoStoreTask.restart();
    }

    public void constructAutoGrab(){
        autoGrabTask.autoStart = false;

        autoGrabTask.addStep(this::preAutoMove);
        // autoGrabTask.addTimedStep(()-> sweepWithPix(2), grabTime);
        autoGrabTask.addStep(()->sweepWithPower(1));
        autoGrabTask.addDelay(1000);
        autoGrabTask.addStep(()->sweepWithPower(0));
        autoGrabTask.addStep(()->setGrabPosition(3));
        reverse = false;
        autoGrabTask.addStep(this::postAutoMove);
        autoGrabTask.addStep(()->triggerEvent(Events.grabComplete));
    }

//    public void constructAutoGrab() {
//        autoGrabTask.addStep(()-> sweepWithPower(.5), ()->hasPixels() == 2);
//        autoGrabTask.addDelay(1000);
//        autoGrabTask.addStep(()-> sweepWithPower(0));
//    }

    public void startAutoGrab() {autoGrabTask.restart();}

    public void addAutoGrabToTask(TimedTask task, int time){
        grabTime = time;
        task.addStep(autoGrabTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.grabComplete));
    }

    public void addAutoGrabToTask(TimedTask task, boolean rev, int time){
        reverse = rev;
        grabTime = time;
        task.addStep(autoGrabTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.grabComplete));
    }

    public void constructAutoDock(){
        autoDockTask.autoStart = false;

        autoDockTask.addStep(this::preAutoMove);
        autoDockTask.addStep(()->setGrabPosition(1));
        autoDockTask.addStep(()->setSwingPosition(2));
        autoDockTask.addDelay(500);
        autoDockTask.addStep(()->setSlidePosition(0));
        autoDockTask.addStep(this::postAutoMove);
        autoDockTask.addStep(() -> triggerEvent(Lifter.Events.dockComplete));
    }

    public void startAutoDock() {autoDockTask.restart();}

    public void addAutoDockToTask(TimedTask task){
        task.addStep(autoDockTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.dockComplete));
    }

    public void constructAutoDrop(){
        autoDropTask.autoStart = false;

        autoDropTask.addStep(this::preAutoMove);
        autoDropTask.addStep(()->setSlidePosition(pixToPos[pix]));
        autoDropTask.addDelay(1000);
        autoDropTask.addTimedStep(()->setSwingPosition(1),1000);
        autoDropTask.addStep(()->setGrabPosition(4));
        autoDropTask.addStep(this::postAutoMove);
        autoDropTask.addStep(() -> triggerEvent(Lifter.Events.dropComplete));
    }

    public void startAutoDrop(){
        autoDropTask.restart();
    }

    public void addAutoDropToTask(TimedTask task){
        task.addStep(autoDropTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.dropComplete));
    }

    public void constructFinishDrop(){
        finishDropTask.autoStart = false;

        finishDropTask.addStep(this::preAutoMove);
        finishDropTask.addStep(()->setGrabPosition(1));
        // dont need this in auto
      //  finishDropTask.addStep((()->
      //          drive.addController("Move to closer pixel drop position", (control) -> control.power = control.power.addY(-.1))));
      //  finishDropTask.addStep(()->setGrabPosition(1));
        finishDropTask.addStep(this::postAutoMove);
        finishDropTask.addStep(()->triggerEvent(Events.finishDropComplete));
    }

    public void startFinishDrop(){
        finishDropTask.restart();
    }

    public void addFinishDropToTask(TimedTask task){
        task.addStep(finishDropTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.finishDropComplete));
    }


    public void doDummyRanging(DriveControl control) {}

    public void setRanging(int doRanging){
        ranging = doRanging;
        switch(ranging){
            case 0:
                doTagRange = false;
                break;
            case 1:
                doTagRange = true;
                break;
        }
    }

    public void doTagRanging(DriveControl control) {
        final double desiredDistance = 7.5;
        final double xPower = 0.01;
        final double yPower = 0.02;

        if (tag.desiredTag != null) {
            inCenter = tag.desiredTag.ftcPose.x > -2 && tag.desiredTag.ftcPose.x < 2;
             inRange = tag.desiredTag.ftcPose.range <= 20;
             currentDist = tag.desiredTag.ftcPose.range;
             double xDist = tag.desiredTag.ftcPose.x;
             atTag = tag.desiredTag.ftcPose.range <= 8.4;
             tooClose = tag.desiredTag.ftcPose.range <= 8.0;

//           if (inRange) {
//                    if(tag.targetFound) { //are we seeing the tag we want?
//                        if (!inCenter) // tag to the right (x is positive) tag to the left (x is negative)
//                            control.power = control.power.addX(xDist * xPower);
//                        else
//                            doTagCenter = false;
//                    }
//            }
           if(doTagRange && inRange){
               if (tag.targetFound) {
                   if (!atTag)
                       control.power = control.power.addY((currentDist - desiredDistance) * -yPower);
                   else {
                       doTagRange = false;
                   }
               }
           }
           else if (tooClose){
               drive.stopRobot();
               doTagRange = false;
               doTagCenter = false;
           }
        }
        else{
            inCenter = false;
            inRange = false;
            currentDist = 0;
            atTag = false;
            tooClose = false;
            doTagRange = false;
            doTagCenter = false;
        }
    }

//    public void watchPixelBucket() {
//        if(getBottomPixelDist() < 2){
//            led.setBottomGroup(TRUE);
//        } else {
//            led.setBottomGroup(FALSE);
//        }
//        if(getTopPixelDist() < 2) {
//            led.setTopGroup(TRUE);
//        } else {
//            led.setTopGroup(FALSE);
//        }
//
//    }

    @Override
    public void onInit() {
        constructAutoDrop();
        constructAutoDock();
        constructAutoGrab();
        constructAutoHome();
        constructAutoArm();
        constructAutoStore();
        constructFinishDrop();

        //setSweepPosition(0);
        setSwingPosition(2);
        setLaunchAngle(2);
        setLaunchState(0);
    }

    @Override
    public void onBeanLoad() {
    }

    @Override
    public void onRun(IntakeControl control) { //TODO separate keeping Slider motor position from onRun
        // slideWithPower(control.sliderPower,false);
        sweepWithPower(control.sweeperPower);
        setGrabPosition(control.grabberPosition);
        robotLiftWithPower(control.robotLiftPosition);
        setLaunchState(control.launchState);
        //watchPixelBucket();
        // setSwingPosition(control.swingPosition);
        setLeds(hasPixels());

//        parent.opMode.telemetry.addData("lifter pos", getRobotLiftPosition());
//        parent.opMode.telemetry.addData("how many pixels", hasPixels());
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
        led = getBeanManager().getBestMatch(Led.class, false);
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
        public final static String autoArm = "auto arm";
        public final static String autoStore = "auto store";
        public final static String autoFinishDrop = "auto finish drop";
    }

    public static final class Events {
        public static final String dockComplete = "DOCK_COMPLETE";
        public static final String grabComplete = "GRAB_COMPLETE";
        public static final String dropComplete = "DROP_COMPlETE";
        public static  final String homeComplete = "HOME_COMPLETE";
        public static final String armComplete = "ARM_COMPLETE";
        public static final String storeComplete = "STORE_COMPLETE";
        public static final String finishDropComplete = "DROP_FINISH_COMPLETE";
    }
}

