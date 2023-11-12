package org.firstinspires.ftc.teamcode.parts.positiontracker.odometry;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTicket;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;

import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.AngleMath;
import om.self.ezftc.utils.Vector2;
import om.self.ezftc.utils.Vector3;
import om.self.ezftc.utils.VectorMath;

public class Odometry extends LoopedPartImpl<PositionTracker, OdometrySettings, OdometryHardware> {
    int lastLeftYPos, lastRightYPos, lastXPos;
    double cumulativeDistance = 0;

    double odoAngle = 0;
    double cumulativeOdoAngle = 0; //TESTING
    double lastImuAngle =  0;

    public Odometry(PositionTracker parent, OdometrySettings settings, OdometryHardware hardware) {
        super(parent, "odometry");
        setConfig(settings, hardware);
    }

    public Odometry(PositionTracker parent) {
        super(parent, "odometry");
        //TODO make default
        setConfig(OdometrySettings.makeForOdoBot(), OdometryHardware.makeForOdoBot(parent.parent.opMode.hardwareMap));
    }

    private double getAngleFromDiff(int leftYDiff, int rightYDiff){
        return (leftYDiff - rightYDiff) * 360 / getSettings().ticksPerRotation;
    }

    @Override
    public void onRun() {

        parent.parent.opMode.telemetry.addData("y odo dist", (getHardware().leftYWheel.getCurrentPosition() + getHardware().rightYWheel.getCurrentPosition()) / 2.0);
        parent.parent.opMode.telemetry.addData("cumulativeDistance", cumulativeDistance);

        int currLeftY = getHardware().leftYWheel.getCurrentPosition();
        int currRightY = getHardware().rightYWheel.getCurrentPosition();
        int currX = getHardware().XWheel.getCurrentPosition();

        parent.parent.opMode.telemetry.addData("left y", currLeftY);
        parent.parent.opMode.telemetry.addData("right y", currRightY);
        parent.parent.opMode.telemetry.addData("middle x", currX);

        int leftYDiff = currLeftY - lastLeftYPos;
        int rightYDiff = currRightY - lastRightYPos;
        int XDiff = currX - lastXPos;

        parent.parent.opMode.telemetry.addData("left y diff", leftYDiff);
        parent.parent.opMode.telemetry.addData("right y diff", rightYDiff);
        parent.parent.opMode.telemetry.addData("x", XDiff);

        Vector3 pos = parent.getCurrentPosition();

        odoAngle = AngleMath.scaleAngle(getAngleFromDiff(leftYDiff, rightYDiff) + odoAngle);
        cumulativeOdoAngle = AngleMath.scaleAngle(getAngleFromDiff(leftYDiff, rightYDiff) + cumulativeOdoAngle);

        double imuAng = parent.getImuAngle();
        boolean imuAccurate = Math.abs(imuAng - lastImuAngle) < 0.5;
        if(imuAccurate){
            odoAngle = imuAng;
        }

        lastImuAngle = imuAng;

        parent.parent.opMode.telemetry.addData("imu accurate", imuAccurate);
        parent.parent.opMode.telemetry.addData("odo angle", odoAngle);
        parent.parent.opMode.telemetry.addData("cumulitave angle", cumulativeOdoAngle);

        double angle = odoAngle;

        double XMove = XDiff / getSettings().ticksPerInch;
        double YMove = (leftYDiff + rightYDiff) / (2 * getSettings().ticksPerInch);

        cumulativeDistance += YMove;

        parent.addPositionTicket(Odometry.class, new PositionTicket(VectorMath.translateAsVector2(pos.withZ(imuAng), XMove, YMove), new Vector2(XMove, YMove)));

        lastLeftYPos = currLeftY;
        lastRightYPos = currRightY;
        lastXPos = currX;
    }

    @Override
    public void onBeanLoad() {

    }

    @Override
    public void onInit() {
    }

    @Override
    public void onStart() {
        getHardware().XWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().leftYWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        getHardware().rightYWheel.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lastXPos = getHardware().XWheel.getCurrentPosition();
        lastLeftYPos = getHardware().leftYWheel.getCurrentPosition();
        lastRightYPos = getHardware().rightYWheel.getCurrentPosition();

        odoAngle = parent.getCurrentPosition().Z;
        lastImuAngle = parent.getImuAngle();
    }

    @Override
    public void onStop() {
    }

    public void lower(){
        getHardware().leftYServo.setPosition(getSettings().leftYServoDown);
        getHardware().rightYServo.setPosition(getSettings().rightYServoDown);
        getHardware().XWheelServo.setPosition(getSettings().XServoDown);
    }

    public void raise(){
        getHardware().leftYServo.setPosition(getSettings().leftYServoUp);
        getHardware().rightYServo.setPosition(getSettings().rightYServoUp);
        getHardware().XWheelServo.setPosition(getSettings().XServoUp);
    }
}
