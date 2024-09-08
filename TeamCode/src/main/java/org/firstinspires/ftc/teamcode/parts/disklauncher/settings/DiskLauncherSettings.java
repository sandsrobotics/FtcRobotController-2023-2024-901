package org.firstinspires.ftc.teamcode.parts.disklauncher.settings;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;

public class DiskLauncherSettings {
    public final double ticksPerRev;
    public final double gearRatio;
    public final double spinMultiplier;
    public final double targetWheelRpm;
    public final RevBlinkinLedDriver.BlinkinPattern defaultLedPattern;
    public final PIDFCoefficients launcherMotorPID;

    public DiskLauncherSettings(double ticksPerRev, double gearRatio, double spinMultiplier, double targetWheelRpm,RevBlinkinLedDriver.BlinkinPattern defaultLedPattern, PIDFCoefficients launcherMotorPID) {
        this.ticksPerRev = ticksPerRev;
        this.gearRatio = gearRatio;
        this.spinMultiplier = spinMultiplier;
        this.targetWheelRpm = targetWheelRpm;
        this.defaultLedPattern = defaultLedPattern;
        this.launcherMotorPID = launcherMotorPID;
    }

    public static DiskLauncherSettings makeDefault() {
        return new DiskLauncherSettings(
                28,
                1,
                60 / 28,
                3700,
                RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE,
                new PIDFCoefficients(100,0,0,12.4)
        );
    }
}
