package org.firstinspires.ftc.teamcode.parts.drive.settings;

import com.qualcomm.robotcore.hardware.Gamepad;

import java.util.Vector;
import java.util.function.Supplier;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.modifiers.LatchedModifier;
import om.self.supplier.suppliers.LatchedSupplier;

public class DriveTeleopSettings {
    public final Supplier<Vector3> powerSupplier;

    public final Supplier<Boolean> stopSupplier;

    public final Supplier<Boolean> midModeSupplier;
    public final double midModeSpeed;

    public final Supplier<Boolean> slowModeSupplier;
    public final double slowModeSpeed;

    public final Supplier<Boolean> invertSupplier;

    public DriveTeleopSettings(Supplier<Vector3> powerSupplier, Supplier<Boolean> stopSupplier, Supplier<Boolean> midModeSupplier, double midModeSpeed, Supplier<Boolean> slowModeSupplier, double slowModeSpeed, Supplier<Boolean> invertSupplier) {
        this.powerSupplier = powerSupplier;
        this.stopSupplier = stopSupplier;
        this.midModeSupplier = midModeSupplier;
        this.midModeSpeed = midModeSpeed;
        this.slowModeSupplier = slowModeSupplier;
        this.slowModeSpeed = slowModeSpeed;
        this.invertSupplier = invertSupplier;
    }

    public static DriveTeleopSettings makeDefault(Robot robot){
        Gamepad gamepad = robot.opMode.gamepad1;

        return new DriveTeleopSettings(
                () -> new Vector3(
                        gamepad.left_stick_x,
                        -gamepad.left_stick_y,
                        gamepad.right_stick_x
                ),
                () -> gamepad.x,
                new LatchedModifier().toSupplier(() -> gamepad.right_bumper),
                0.7,
                //new LatchedModifier().toSupplier(() -> gamepad.b),
                () -> gamepad.right_trigger > 0.5,
                0.5,
                new LatchedModifier(false).toSupplier(() -> gamepad.left_bumper)
        );
    }

    public static DriveTeleopSettings makeForza(Robot robot){
        Gamepad gamepad = robot.opMode.gamepad1;

        return new DriveTeleopSettings(
                () -> new Vector3(
                        -gamepad.right_stick_x,
                        gamepad.left_trigger - gamepad.right_trigger,
                        gamepad.left_stick_x
                ),
                () -> gamepad.x,
                new LatchedModifier().toSupplier(() -> gamepad.right_bumper),
                0.7,
                //new LatchedModifier().toSupplier(() -> gamepad.b),
                () -> false,
                0.5,
                new LatchedModifier(false).toSupplier(() -> gamepad.left_bumper)
        );
    }
}
