package org.firstinspires.ftc.teamcode.parts.intake;

import android.graphics.Color;

import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.apache.commons.math3.geometry.spherical.twod.Edge;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.depricated.lifter.Lifter;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake.hardware.IntakeHardware;
import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeSettings;
import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.ControllablePart;
import om.self.supplier.suppliers.EdgeSupplier;
import om.self.task.core.Group;
import om.self.task.core.TaskEx;
import om.self.task.other.TimedTask;

public class Intake extends ControllablePart<Robot, IntakeSettings, IntakeHardware, IntakeControl>{
    public int slideTargetPosition;
    private int liftTargetPosition;
    public boolean doTagRange = false;
    public boolean doTagCenter = false;
    public int grabTime;
    private boolean armed;
    private Integer id;
    boolean atAutoTag;
    boolean atTeleTag;
    boolean tooClose;
    boolean inAngle;
    boolean inAutoRange;
    boolean run = false;
    boolean runCenter = false;
    boolean inCenter;
    private boolean reverse;
    AprilTag tag;
    Drive drive;
    Led led;
    boolean isTop;
    double motorPower = 0;
    private int pixLine = 0;
    private double backDist;
    private final int[] pixToPos = {1000,1100,1360,1620,1880, 2140, 2400, 2660, 2920, 3180, 3440, 3700, 3960}; //TODO move to settings, need 12 of these
    private final int[] pixLineToPos = {1000, 1200, 1400, 1800, 2200, 2600, 3000};
    private final Group movementTask = new Group("auto movement",getTaskManager());
    private final TimedTask autoDropTask = new TimedTask(TaskNames.autoDrop, movementTask);
    private final TimedTask autoGrabTask = new TimedTask(TaskNames.autoGrab, getTaskManager());
    private final TimedTask autoHomeTask = new TimedTask(TaskNames.autoHome, movementTask);
    private final TimedTask autoDockTask = new TimedTask(TaskNames.autoDock, getTaskManager());
    private final TimedTask autoArmTask = new TimedTask(TaskNames.autoArm, getTaskManager());
    private final TimedTask autoStoreTask = new TimedTask(TaskNames.autoStore, getTaskManager());
    private  final TimedTask finishDropTask = new TimedTask(TaskNames.autoFinishDrop, movementTask);
    private  final TimedTask foundRangeTask = new TimedTask(TaskNames.autoFoundRange, getTaskManager());



    //***** Constructors *****
    public Intake(Robot parent) {
        super(parent, "Slider", () -> new IntakeControl(0, 0, 0,  0, 0,0, 0));
        setConfig(
                IntakeSettings.makeDefault(),
                IntakeHardware.makeDefault(parent.opMode.hardwareMap)
        );
    }

    public Intake(Robot parent, IntakeSettings settings, IntakeHardware hardware){
        super(parent, "slider", () -> new IntakeControl(0,0, 0,  0, 0,0, 0));
        setConfig(settings, hardware);
    }

    private void preAutoMove(){triggerEvent(ControllablePart.Events.stopControllers);}
    private void postAutoMove(){triggerEvent(ControllablePart.Events.startControllers);}

    private void setSlideToHomeConfig(){
        double power = -0.125;

        getHardware().sliderMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);

        getHardware().sliderMotor.setPower(power);
        //getHardware().rightLiftMotor.setPower(power);
    }

    private void setMotorsToRunConfig(){
        getHardware().sliderMotor.setPower(IntakeHardware.slideHoldPower);
        //getHardware().rightLiftMotor.setPower(LifterHardware.liftHoldPower);
        getHardware().sliderMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
    }

    public void slideWithPower(double power, boolean force){
        if(Math.abs(power) < getSettings().minRegisterVal) return;

        if(power < 0)
            power *= getSettings().maxDownSlideSpeed;
        else
            power *= getSettings().maxUpSlideSpeed;

        if(force)
            setSlidePositionUnsafe(getSlidePosition() + (int)power);
        else
            setSlidePosition(getSlidePosition() + (int)power);
    }

    public void sweepWithPower(double power) {
        getHardware().sweeperMotor.setPower(.52 * power);
        if(power != 0) {
            topDist = getHardware().topSensor.getDistance(DistanceUnit.CM);
            botDist = getHardware().botSensor.getDistance(DistanceUnit.CM);
            setLeds2(hasPixels());
        }
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
            case 3:
                getHardware().swingServoLeft.setPosition(getSettings().swingLeftMaxPosition);
                getHardware().swingServoRight.setPosition(getSettings().swingLeftMaxPosition);
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

    private int currentSlidePos;
    private int currentLiftPos;
    private double topDist;
    private double botDist;

    public void setLiftToTop(){
        setSlidePosition(getSettings().maxSlidePosition);
    }
    public void setLiftToBottom(){
        setSlidePosition(getSettings().minSlidePosition);
    }
    public int getSlidePosition(){
        return currentSlidePos;
    }
    public int getRobotLiftPosition(){return currentLiftPos;}
    public double getTopPixelDist() { return topDist;}
    public double getBottomPixelDist() { return botDist;}
    public double getBackDist() { return backDist;}

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
        if (getTopPixelDist() < 2.5 && getBottomPixelDist() < 2.5)
            return 2;
        else if (getTopPixelDist() > 2.5 && getBottomPixelDist() < 2.5)
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

    public int getLedBucket(RevColorSensorV3 sensor) {
        float[] hsvValues = new float[3];
        NormalizedRGBA colors = sensor.getNormalizedColors();
        Color.colorToHSV(colors.toColor(), hsvValues);
        //parent.opMode.telemetry.addData("Hue", hsvValues[0]);
        int hue = (int) hsvValues[0];
        if (hue < 120) return 1; //yellow
        if (hue >= 120 && hue <= 150) return 2; //green
        if (hue > 180) return 3; //purple
        else return 4; //white
    }

    public void setLeds2(int pixels){
        if (pixels == 1) {
            led.setBottomGroup2(getLedBucket(getHardware().botSensor));
            led.setTopGroup2(0);
        } else if(pixels == 2) {
            led.setBottomGroup2(getLedBucket(getHardware().botSensor));
            led.setTopGroup2(getLedBucket(getHardware().topSensor));
        } else {
            led.setBottomGroup2(0);
            led.setTopGroup2(0);
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

    public void  setGrabPosition(int position) {
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

    public int getPix(){return pixLine;}

    public void setPix(int pix) {
        if(pix > getSettings().maxPixLine) pix = getSettings().maxPixLine;
        else if(pix <= 0) pix = 0;
        this.pixLine = pix;
    }

    public void robotLiftWithPower(int power) {
        motorPower = power;

        if (power < 0) { // going down
            isTop = false;
            if (getHardware().liftLowLimitSwitch.getState())
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
            getHardware().sliderMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
            //getHardware().rightLiftMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

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
        autoArmTask.addTimedStep(()->robotLiftWithPower(1), 1500);;
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
        autoGrabTask.addTimedStep(()->sweepWithPower(reverse ? 1 : -1), ()->hasPixels() == 2, 2000); // tjk
        autoGrabTask.addStep(()->sweepWithPower(0));
        autoGrabTask.addStep(this::postAutoMove);
        autoGrabTask.addStep(()->triggerEvent(Events.grabComplete));
    }

//    public void constructAutoGrab() {
//        autoGrabTask.addStep(()-> sweepWithPower(.5), ()->hasPixels() == 2);
//        autoGrabTask.addDelay(1000);
//        autoGrabTask.addStep(()-> sweepWithPower(0));
//    }

    public void startAutoGrab() {autoGrabTask.restart();}

    public void addAutoGrabToTask(TaskEx task){
        task.addStep(autoGrabTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.grabComplete));
    }

    public void addAutoGrabToTask(TaskEx task, boolean reverse){
        task.addStep((Runnable) () -> this.reverse = reverse);
        addAutoGrabToTask(task);
    }

    public void addAutoGrabToTask(TaskEx task, boolean reverse, int grabTime){
        task.addStep(()-> this.grabTime = grabTime);
        task.addStep((Runnable) () -> this.reverse = reverse);
        addAutoGrabToTask(task);
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

    public void addAutoDockToTask(TaskEx task){
        task.addStep(autoDockTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.dockComplete));
    }

    public void constructAutoDrop(){
        autoDropTask.autoStart = false;

//        autoDropTask.addStep(this::preAutoMove);
        autoDropTask.addStep(()->setGrabPosition(4));
        autoDropTask.addStep(()->setSlidePosition(pixLineToPos[pixLine]));
        autoDropTask.addDelay(500);
        autoDropTask.addTimedStep(()->setSwingPosition(1),1000);
        autoDropTask.addStep(()-> setLeds2(hasPixels()));
//        autoDropTask.addStep(this::postAutoMove);
        autoDropTask.addStep(() -> triggerEvent(Lifter.Events.dropComplete));
    }

    public void startAutoDrop(){
        autoDropTask.restart();
    }

    public void addAutoDropToTask(TaskEx task){
        task.addStep(autoDropTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.dropComplete));
    }

    public void constructFinishDrop(){
        finishDropTask.autoStart = false;

        finishDropTask.addStep(this::preAutoMove);
        finishDropTask.addStep(()->setGrabPosition(1)); //change to 2 when grabbing extra pix in auto
//        finishDropTask.addDelay(1000);
//        finishDropTask.addStep(()->setSlidePosition(pixLineToPos[pixLine + 1]));
//        finishDropTask.addDelay(1000);
//          finishDropTask.addStep(()->setGrabPosition(1));
//          finishDropTask.addDelay(2000);
        finishDropTask.addStep(this::postAutoMove);
        finishDropTask.addStep(()->triggerEvent(Events.finishDropComplete));
    }

    public void startFinishDrop(){
        finishDropTask.restart();
    }

    public void addFinishDropToTask(TaskEx task){
        task.addStep(finishDropTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.finishDropComplete));
    }

    public void constructFoundRange(){
        foundRangeTask.autoStart = false;

        foundRangeTask.addStep(()-> run = true);

        foundRangeTask.addStep(()->triggerEvent(Events.foundRangeComplete));
    }

    public void startFoundRange(){ foundRangeTask.restart();}

    public void addFoundRangeToTask(TimedTask task){
        task.addStep(foundRangeTask::restart);
        task.waitForEvent(eventManager.getContainer(Events.foundRangeComplete));
    }

    public void setRunCenter(boolean run) { runCenter = run; }

    public void doDummyRanging(DriveControl control) {}

    public void setRanging(int doRanging){
        switch(doRanging){
            case 0:
                doTagRange = false;
                break;
            case 1:
                doTagRange = true;
                break;
        }
    }

    public void setCentering(int doCentering){
        switch(doCentering){
            case 0:
                doTagCenter = false;
                break;
            case 1:
                doTagCenter = true;
                break;
        }
    }


    public void doTagRanging(DriveControl control) {
        final double desiredAutoDistance = 7.65;
        final double desiredTeleDistance = 7.65;
        final double xPower = 0.03;
        final double yPower = 0.05;
        final double zPower = 0.01;
        final double yAutoPower = 0.03;
        atAutoTag = getBackDist() <= desiredAutoDistance;
        atTeleTag = getBackDist() <= desiredTeleDistance;
        tooClose = getBackDist() <= 6;
        inAutoRange = getBackDist() <= 45;

        if(doTagCenter) {
            if (tag.desiredTag != null) {
                id = tag.desiredTag.id;
                switch(id){
                    case 2:
                        tag.setDesiredTag(2);
                    case 8:
                        tag.setDesiredTag(8);
                    case 5:
                        tag.setDesiredTag(5);
                    case 9:
                        tag.setDesiredTag(9);

                        inCenter = tag.desiredTag.ftcPose.x > -1 && tag.desiredTag.ftcPose.x < 1;
                        inAngle = tag.desiredTag.ftcPose.bearing > -1 && tag.desiredTag.ftcPose.bearing < 1;
                        double xDist = tag.desiredTag.ftcPose.x;
                        double bearing = tag.desiredTag.ftcPose.bearing;

                        if(tag.targetFound) {
                            if (!inAngle)
                                control.power = control.power.addZ(bearing * zPower);
                            else if (!inCenter) // tag to the right (x is positive) tag to the left (x is negative)
                                control.power = control.power.addX(xDist * -xPower);
                            else {
                                tag.setDesiredTag(-1);
                                doTagCenter = false;
                            }
                        }
                        break;
                }
            }
        }
            else if (doTagRange) {
                if (!atTeleTag) {
                    control.power = control.power.addY((getBackDist() - desiredTeleDistance) * -yPower);
                } else {
                    doTagRange = false;
                }
            } else if (tag.DESIRED_TAG_ID != -1) {
                if (run && inAutoRange) {
                    if (!atAutoTag) {
                        control.power = control.power.addY((getBackDist() - desiredAutoDistance) * -yAutoPower);
                    } else {
                        tag.setDesiredTag(-1);
                        run = false;
                    }
                }
                else if(runCenter && tag.targetFound){

                    inCenter = tag.desiredTag.ftcPose.x > -1 && tag.desiredTag.ftcPose.x < 1;
                    double xDist = tag.desiredTag.ftcPose.x;

                    if (!inCenter) // tag to the right (x is positive) tag to the left (x is negative)
                        control.power = control.power.addX(xDist * -xPower);
                    else {
                        tag.setDesiredTag(-1);
                        runCenter = false;
                    }
                 }
            }
//            else if (tooClose){
//               drive.stopRobot();
//               doTagRange = false;
//               doTagCenter = false;
//           }
//        else{
//            inCenter = false;
//            inRange = false;
//            currentDist = 0;
//            atTag = false;
//            tooClose = false;
//            doTagRange = false;
//            doTagCenter = false;
//        }
        }



    public void setSweepPosition(int position) {
        switch (position) {
            case 1:
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoDownPosition);
                break;
            case 2:
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoStackPosition);
                break;
            case 3:
                getHardware().sweepLiftServo.setPosition(getSettings().sweepLiftServoStackTopPosition);
                break;
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
        constructFoundRange();

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
//        watchPixelBucket();
//         setSwingPosition(control.swingPosition);
        setSweepPosition(control.sweepLiftPosition);
        setRanging(control.ranging);
//        setCentering(control.centering);

        if(doTagRange || run)
          backDist = getHardware().backSensor.getDistance(DistanceUnit.INCH);
        currentSlidePos = getHardware().sliderMotor.getCurrentPosition();
        currentLiftPos = getHardware().robotLiftMotor.getCurrentPosition();
//        parent.opMode.telemetry.addData("lifter pos", getRobotLiftPosition());
//        parent.opMode.telemetry.addData("y:", new EdgeSupplier(()-> parent.opMode.gamepad1.y).getRisingEdgeSupplier());
//        parent.opMode.telemetry.addData("x:", new EdgeSupplier(()-> parent.opMode.gamepad1.x).getRisingEdgeSupplier());
        //parent.opMode.telemetry.addData("Top Pixel (cm)", getTopPixelDist());
        //parent.opMode.telemetry.addData("Bottom Pixel (cm)", getBottomPixelDist());
//        parent.opMode.telemetry.addData("Back Board (In)", getBackDist());
//        parent.opMode.telemetry.addData("ranging", run);
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
        setSweepPosition(0);
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
        public final static String autoFoundRange = "auto found range";
    }

    public static final class Events {
        public static final String dockComplete = "DOCK_COMPLETE";
        public static final String grabComplete = "GRAB_COMPLETE";
        public static final String dropComplete = "DROP_COMPlETE";
        public static  final String homeComplete = "HOME_COMPLETE";
        public static final String armComplete = "ARM_COMPLETE";
        public static final String storeComplete = "STORE_COMPLETE";
        public static final String finishDropComplete = "DROP_FINISH_COMPLETE";
        public static final String foundRangeComplete = "FOUND_RANGE_COMPLETE";
    }
}

