package org.firstinspires.ftc.teamcode.parts.intake.settings;

import com.qualcomm.robotcore.hardware.Gamepad;

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

    public IntakeTeleopSettings(Supplier<Integer> heightSpeedSupplier, Supplier<Boolean> sliderTopSupplier, Supplier<Boolean> sliderBottomSupplier, Supplier<Float> sweepSpeedSupplier, Supplier<Integer> sweepLiftSupplier) {
        this.heightSpeedSupplier = heightSpeedSupplier;
        this.sliderBottomSupplier = sliderBottomSupplier;
        this.sliderTopSupplier = sliderTopSupplier;
        this.sweepSpeedSupplier = sweepSpeedSupplier;
        this.sweepLiftSupplier = sweepLiftSupplier;
    }

    public static IntakeTeleopSettings makeDefault(Robot robot){
        Gamepad gamepad = robot.opMode.gamepad1;
        //Gamepad gamepad2 = robot.opMode.gamepad2;
        EdgeSupplier sliderTop = new EdgeSupplier();
        sliderTop.setBase(() -> gamepad.y);
        EdgeSupplier sliderBottom = new EdgeSupplier();
        sliderBottom.setBase(() -> gamepad.a);

        return new IntakeTeleopSettings(
            () -> gamepad.dpad_down ? -1 : gamepad.dpad_up ? 1 : 0,
            sliderTop::isRisingEdge,
            sliderBottom::isRisingEdge,
            () -> gamepad.left_trigger - gamepad.right_trigger,
            () -> gamepad.dpad_right ? 1 : gamepad.dpad_left ? 2 : 0);
    }
}