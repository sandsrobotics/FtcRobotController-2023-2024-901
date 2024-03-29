package org.firstinspires.ftc.teamcode.parts.intake.settings;

public class IntakeSettings {
    public final int minSlidePosition;
    public final int maxSlidePosition;
    public final int maxDownSlideSpeed;
    public final int maxUpSlideSpeed;
    public final double minRegisterVal;
    public final double tolerance;
    public final double sweepLiftServoStorePosition;
    public final double sweepLiftServoDownPosition;
    public final double sweepLiftServoStackPosition;
    public final int maxDownLiftSpeed;
    public final int maxUpLiftSpeed;
    public final int maxLiftPosition;
    public final int minLiftPosition;
    public final double grabberOpenPosition;
    public final double grabberGripTwoPosition;
    public final double grabberGripOnePosition;
    public final double swingLeftSafePosition;
    public final double swingRightSafePosition;
    public final double swingLeftDropPosition;
    public final double swingRightDropPosition;
    public final int maxPix;
    public final int maxPixLine;
    public final double launchAngleStorePosition;
    public final double launchAngleArmPosition;
    public final double launchReleaseLockPosition;
    public final double launchReleaseUnlockPosition;
    public final double grabberPrimePosition;
    public final double swingLeftMaxPosition;
    public final double sweepLiftServoStackTopPosition;
    public final double swingLeftActualMaxPosition;



    public IntakeSettings(int minSlidePosition, int maxSlidePosition, int maxDownSlideSpeed, int maxUpSlideSpeed, double minRegisterVal, int maxUpLiftSpeed,
                          int maxDownLiftSpeed, double tolerance, double sweepLiftServoDownPosition, double sweepLiftServoStackPosition,
                          double sweepLiftServoStorePosition, int maxLiftPosition, int minLiftPosition, double grabberOpenPosition, double grabberGripTwoPosition,
                          double grabberGripOnePosition, double swingLeftSafePosition, double swingRightSafePosition, double swingLeftDropPosition, double swingRightDropPosition, int maxPix, int maxPixLine, double launchAngleStorePosition,
                          double launchAngleArmPosition, double launchReleaseLockPosition, double launchReleaseUnlockPosition, double grabberPrimePosition, double swingLeftMaxPosition, double sweepLiftServoStackTopPosition, double swingLeftActualMaxPosition) {
        this.minSlidePosition = minSlidePosition;
        this.maxSlidePosition = maxSlidePosition;
        this.maxDownSlideSpeed = maxDownSlideSpeed;
        this.maxUpSlideSpeed = maxUpSlideSpeed;
        this.minRegisterVal = minRegisterVal;
        this.maxDownLiftSpeed = maxDownLiftSpeed;
        this.maxUpLiftSpeed = maxUpLiftSpeed;
        this.tolerance = tolerance;
        this.sweepLiftServoStorePosition = sweepLiftServoStorePosition;
        this.sweepLiftServoDownPosition = sweepLiftServoDownPosition;
        this.sweepLiftServoStackPosition = sweepLiftServoStackPosition;
        this.maxLiftPosition = maxLiftPosition;
        this.minLiftPosition = minLiftPosition;
        this.grabberOpenPosition = grabberOpenPosition;
        this.grabberGripTwoPosition = grabberGripTwoPosition;
        this.grabberGripOnePosition = grabberGripOnePosition;
        this.swingLeftSafePosition = swingLeftSafePosition;
        this.swingRightSafePosition = swingRightSafePosition;
        this.swingLeftDropPosition = swingLeftDropPosition;
        this.swingRightDropPosition = swingRightDropPosition;
        this.maxPix = maxPix;
        this.maxPixLine = maxPixLine;
        this.launchAngleStorePosition = launchAngleStorePosition;
        this.launchAngleArmPosition = launchAngleArmPosition;
        this.launchReleaseLockPosition = launchReleaseLockPosition;
        this.launchReleaseUnlockPosition = launchReleaseUnlockPosition;
        this.grabberPrimePosition = grabberPrimePosition;
        this.swingLeftMaxPosition = swingLeftMaxPosition;
        this.sweepLiftServoStackTopPosition = sweepLiftServoStackTopPosition;
        this.swingLeftActualMaxPosition = swingLeftActualMaxPosition;
    }

    // left safe: ,.14 drop: .5 FOR ACTUAL MAX, .6
    // right safe: .87, drop: .48

    public static IntakeSettings makeDefault(){
        return new IntakeSettings(
                0,
                3350,
                300,
                300,
                0.05,
                300,
                300,
                2,
                0.209,
                0.330,
                0.776,
                8699,
                201,
                .95,
                .15,
                .56,
                .158,
                .842,
                .52,
                .48,
                12,
                7,
                .46,
                .59,
                .482,
                .702,
                .42,
                .56,
                .365,
                        .6
                );
    }
}
