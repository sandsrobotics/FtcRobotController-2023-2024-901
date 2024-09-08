package org.firstinspires.ftc.teamcode.parts.disklauncher;

public class DiskLauncherControl {
    boolean conveyerPreset;
    boolean flipPreset;
    boolean launcherPreset;
    boolean flipPresetSafety;
    boolean flipPresetSafetyOverride;

    public DiskLauncherControl(boolean conveyerPreset, boolean flipPreset,
                               boolean launcherPreset, boolean flipPresetSafety,
                               boolean flipPresetSafetyOverride){
        this.conveyerPreset = conveyerPreset;
        this.flipPreset = flipPreset;
        this.flipPresetSafety = flipPresetSafety;
        this.launcherPreset = launcherPreset;
        this.flipPresetSafetyOverride = flipPresetSafetyOverride;
    }
}
