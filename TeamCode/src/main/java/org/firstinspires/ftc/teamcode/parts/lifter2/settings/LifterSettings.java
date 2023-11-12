package org.firstinspires.ftc.teamcode.parts.lifter2.settings;

import om.self.ezftc.utils.RangeDouble;
import om.self.ezftc.utils.RangeInt;

public class LifterSettings {
    //turn servos
    public final RangeDouble turnAngleRange;
    public final RangeDouble turnServoRange;
    public final RangeDouble turnPotentiometerRange;

    public final double rightTurnServoOffset;

    //grabber servo
    public final double grabberServoClosePos;
    public final double grabberServoOpenPos;
    public final double grabberServoWideOpenPos;
    public final int servoCloseToOpenTime; //time in ms

    //motor
    public final RangeInt lifterRange;

    public final double liftHoldPower;
    public final int maxDownSpeed;
    public final int maxUpSpeed;
    public final int tolerance;

    //changed grabberservo open pos from .9 to .86 (wouldn't open all the way during autonomous test)


    public LifterSettings(RangeDouble turnAngleRange, RangeDouble turnServoRange, RangeDouble turnPotentiometerRange, double rightTurnServoOffset, double grabberServoClosePos, double grabberServoOpenPos, double grabberServoWideOpenPos, int servoCloseToOpenTime, RangeInt lifterRange, double liftHoldPower, int maxDownSpeed, int maxUpSpeed, int tolerance) {
        this.turnAngleRange = turnAngleRange;
        this.turnServoRange = turnServoRange;
        this.turnPotentiometerRange = turnPotentiometerRange;
        this.rightTurnServoOffset = rightTurnServoOffset;
        this.grabberServoClosePos = grabberServoClosePos;
        this.grabberServoOpenPos = grabberServoOpenPos;
        this.grabberServoWideOpenPos = grabberServoWideOpenPos;
        this.servoCloseToOpenTime = servoCloseToOpenTime;
        this.lifterRange = lifterRange;
        this.liftHoldPower = liftHoldPower;
        this.maxDownSpeed = maxDownSpeed;
        this.maxUpSpeed = maxUpSpeed;
        this.tolerance = tolerance;
    }

    public static LifterSettings makeDefault(){
        return new LifterSettings(
                null,
                null,
                null,
                0,
                0,
                0,
                0,
                0,
                null,
                0,
                0,
                0,
                0
        );
    }
}
