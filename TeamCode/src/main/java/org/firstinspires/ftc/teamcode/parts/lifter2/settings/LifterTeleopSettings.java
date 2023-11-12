package org.firstinspires.ftc.teamcode.parts.lifter2.settings;

import com.qualcomm.robotcore.hardware.Gamepad;
import org.firstinspires.ftc.teamcode.parts.lifter2.Lifter;

import java.util.function.Supplier;

public class LifterTeleopSettings {
    //basic controls
    public final Supplier<Float> liftSpeedSupplier;
    public final Supplier<Float> turnSpeedSupplier;
    public final Supplier<Lifter.GrabberPosition> grabberPositionSupplier;

    public final Supplier<Boolean> forceCloseSupplier;
    public final Supplier<Boolean> forceliftDownSupplier;
    //presets
    public final Supplier<Boolean> autoGrabSupplier;
    public final Supplier<Boolean> autoDockSupplier;
    public final Supplier<Lifter.PoleHeight> preDropSupplier;
    public final Supplier<Boolean> autoDropSupplier;
    public final Supplier<Boolean> autoHomeSupplier;
    //other
    public final Supplier<Integer> coneChangeSupplier; //change the cone that it docks for

    public LifterTeleopSettings(Supplier<Float> liftSpeedSupplier, Supplier<Float> turnSpeedSupplier, Supplier<Lifter.GrabberPosition> grabberPositionSupplier, Supplier<Boolean> forceCloseSupplier, Supplier<Boolean> forceliftDownSupplier, Supplier<Boolean> autoGrabSupplier, Supplier<Boolean> autoDockSupplier, Supplier<Lifter.PoleHeight> preDropSupplier, Supplier<Boolean> autoDropSupplier, Supplier<Boolean> autoHomeSupplier, Supplier<Integer> coneChangeSupplier) {
        this.liftSpeedSupplier = liftSpeedSupplier;
        this.turnSpeedSupplier = turnSpeedSupplier;
        this.grabberPositionSupplier = grabberPositionSupplier;
        this.forceCloseSupplier = forceCloseSupplier;
        this.forceliftDownSupplier = forceliftDownSupplier;
        this.autoGrabSupplier = autoGrabSupplier;
        this.autoDockSupplier = autoDockSupplier;
        this.preDropSupplier = preDropSupplier;
        this.autoDropSupplier = autoDropSupplier;
        this.autoHomeSupplier = autoHomeSupplier;
        this.coneChangeSupplier = coneChangeSupplier;
    }

    public static LifterTeleopSettings makeDefault(Gamepad main, Gamepad alternate){
        return new LifterTeleopSettings(
                () -> 0f,
                () -> 0f,
                () -> null,
                () -> false,
                () -> false,
                () -> false,
                () -> false,
                () -> null,
                () -> false,
                () -> false,
                () -> 0
        );
    }
}

