package org.firstinspires.ftc.teamcode.parts.disklauncher.settings;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.parts.intake.settings.IntakeTeleopSettings;

import java.util.function.Supplier;

import om.self.ezftc.core.Robot;
import om.self.supplier.core.Utils;

public class DiskLauncherTeleopSettings {
    public final Supplier<Boolean> flipPresetSupplier;
    public final Supplier<Boolean> flipPresetSupplierSafety;
    public final Supplier<Boolean> flipPresetSupplierSafetyOverride;
    public final Supplier<Boolean> launcherPresetSupplier;
    public final Supplier<Boolean> conveyerPresetSupplier;

    public DiskLauncherTeleopSettings(Supplier<Boolean> flipPresetSupplier, Supplier<Boolean> flipPresetSupplierSafety,
                                      Supplier<Boolean> flipPresetSupplierSafetyOverride, Supplier<Boolean> launcherPresetSupplier,
                                      Supplier<Boolean> conveyerPresetSupplier){
        this.flipPresetSupplier = flipPresetSupplier;
        this.flipPresetSupplierSafety = flipPresetSupplierSafety;
        this.flipPresetSupplierSafetyOverride = flipPresetSupplierSafetyOverride;
        this.launcherPresetSupplier = launcherPresetSupplier;
        this.conveyerPresetSupplier = conveyerPresetSupplier;
    }

    public static DiskLauncherTeleopSettings makeDefault(Robot robot){
        Gamepad gamepad = robot.opMode.gamepad1;
        Gamepad gamepad2 = robot.opMode.gamepad2;

        return new DiskLauncherTeleopSettings(
            ()-> gamepad.right_trigger > 0.1,
            ()-> gamepad2.right_trigger > 0.1,
            ()-> gamepad.left_bumper,
            ()-> gamepad.b || gamepad2.b,
            ()-> gamepad.a || gamepad2.a
        );
    }
}
