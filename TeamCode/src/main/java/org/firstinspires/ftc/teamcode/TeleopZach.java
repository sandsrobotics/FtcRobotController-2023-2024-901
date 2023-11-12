package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
//import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.teamcode.parts.lifter.Lifter;
import org.firstinspires.ftc.teamcode.parts.lifter.LifterTeleop;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;

import java.text.DecimalFormat;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;
import om.self.task.other.TimedTask;

/**
 * This file contains an example of an iterative (Non-Linear) "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode
 * class is instantiated on the Robot Controller and executed.
 *
 * This particular OpMode just executes a basic Tank Drive Teleop for a two wheeled robot
 * It includes all the skeletal structure that all iterative OpModes contain.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="Teleop Zach", group="Linear Opmode")
public class TeleopZach extends LinearOpMode {
    double tileSide = 23.5;
    Vector3 fieldStartPos = new Vector3(-36,63,90);

    public Vector3 tiletoField(Vector3 p){
        return new Vector3(p.X * tileSide, p.Y * tileSide, p.Z);
    }

    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }

    @Override
    public void runOpMode() {
        long start;

        Robot r = new Robot(this);
        new BulkRead(r);
        Drive d = new Drive(r);
        new DriveTeleop(d);
        //new HeaderKeeper(d);
        PositionTracker pt = new PositionTracker(r);

//        Slamra s = new Slamra(pt);
        EncoderTracker et = new EncoderTracker(pt);

        Lifter l = new Lifter(r);
        l.dockDelay = 600;
        new LifterTeleop(l);


        DecimalFormat df = new DecimalFormat("#0.0");
        r.init();

        while (!isStarted()) {}

        r.start();

        //l.startAutoHome();

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            r.run();
            telemetry.addData("limit hit", !l.getHardware().limitSwitch.getState());
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("tilt position", l.getCurrentTurnPosition());
            telemetry.addData("is grabber closed", l.isGrabberClosed());
            telemetry.addData("right servo offset", df.format(l.getSettings().rightTurnServoOffset));
            telemetry.addData("lift position:",df.format(l.getLiftPosition()));
            telemetry.addData("Ultra [Left : Mid : Right]", "["
                    + df.format(l.getLeftUltra()) + " : "
                    + df.format(l.getMidUltra()) + " : "
                    + df.format(l.getRightUltra()) +"]");


            if(gamepad1.dpad_down) telemetry.addData("tasks", r.getTaskManager());
            if(gamepad1.dpad_down) telemetry.addData("events", r.getEventManager());
            r.opMode.telemetry.addData("time", System.currentTimeMillis() - start);

            telemetry.update();
        }
        r.stop();
    }
}
