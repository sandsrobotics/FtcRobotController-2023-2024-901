package org.firstinspires.ftc.teamcode.parts.intake.settings;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.parts.intake.Intake;

import java.util.concurrent.Callable;
import java.util.function.Supplier;
import om.self.ezftc.core.Robot;
import om.self.supplier.suppliers.EdgeSupplier;

public class IntakeTeleopSettings {
    public final Supplier<Integer> heightSpeedSupplier;
    public final Supplier<Boolean> sliderTopSupplier;
    public final Supplier<Boolean> sliderBottomSupplier;
    public final Supplier<Float> sweepSpeedSupplier;
    public final Supplier<Integer> sweepLiftSupplier;
    public final Supplier<Integer> robotLiftSupplier;
    public final Supplier<Integer> grabberSupplier;
    public final Supplier<Integer> swingSupplier;
    public final Supplier<Integer> pixChangeSupplier;
    public final Supplier<Boolean> autoDropSupplier;
    public final Supplier<Boolean> autoDockSupplier;
    public final Supplier<Integer> launchAngleSupplier;
    public final Supplier<Integer> launchReleaseSupplier;



    public IntakeTeleopSettings(Supplier<Integer> heightSpeedSupplier, Supplier<Boolean> sliderTopSupplier,
                                Supplier<Boolean> sliderBottomSupplier, Supplier<Float> sweepSpeedSupplier,
                                Supplier<Integer> sweepLiftSupplier, Supplier<Integer> robotLiftSupplier,
                                Supplier<Integer> grabberSupplier, Supplier<Integer> swingSupplier,
                                Supplier<Integer> pixChangeSupplier, Supplier<Boolean> autoDropSupplier,
                                Supplier<Boolean> autoDockSupplier, Supplier<Integer> launchAngleSupplier,
                                Supplier<Integer> launchReleaseSupplier) {
        this.heightSpeedSupplier = heightSpeedSupplier;
        this.sliderBottomSupplier = sliderBottomSupplier;
        this.sliderTopSupplier = sliderTopSupplier;
        this.sweepSpeedSupplier = sweepSpeedSupplier;
        this.sweepLiftSupplier = sweepLiftSupplier;
        this.robotLiftSupplier = robotLiftSupplier;
        this.grabberSupplier = grabberSupplier;
        this.swingSupplier = swingSupplier;
        this.pixChangeSupplier = pixChangeSupplier;
        this.autoDropSupplier = autoDropSupplier;
        this.autoDockSupplier = autoDockSupplier;
        this.launchAngleSupplier = launchAngleSupplier;
        this.launchReleaseSupplier = launchReleaseSupplier;
    }

    public static IntakeTeleopSettings makeDefault(Robot robot){
        Gamepad gamepad = robot.opMode.gamepad1;
        Gamepad gamepad2 = robot.opMode.gamepad2;

        EdgeSupplier sliderTop = new EdgeSupplier();
        sliderTop.setBase(() -> gamepad.y);

        EdgeSupplier sliderBottom = new EdgeSupplier();
        sliderBottom.setBase(() -> gamepad.a);

        EdgeSupplier autoDrop = new EdgeSupplier();
        autoDrop.setBase(() -> gamepad2.dpad_up);

        EdgeSupplier autoDock = new EdgeSupplier();
        autoDock.setBase(()-> gamepad2.dpad_down);

        return new IntakeTeleopSettings(
            () -> gamepad.dpad_down ? -1 : gamepad.dpad_up ? 1 : 0,
            sliderTop::isRisingEdge,
            sliderBottom::isRisingEdge,
            ()-> gamepad.left_trigger - gamepad.right_trigger,
            () -> 0,
            () -> gamepad.left_bumper ? -1 : gamepad.right_bumper ? 1 : 0,
            () -> gamepad2.y ? 1 : gamepad2.b ? 2 : gamepad2.a ? 3 : 0,
            () -> gamepad.dpad_right ? 1 : gamepad.dpad_left ? 2 : 0,
            () -> gamepad2.left_bumper ? -1 : gamepad2.right_bumper ? 1 : 0,
            autoDrop::isRisingEdge,
            autoDock::isRisingEdge,
            () -> gamepad2.dpad_right ? 1 : gamepad2.dpad_left ? 2 : 0,
            () -> gamepad2.x ? 1 : 0
        );
    }
}