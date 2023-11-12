package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.drive.headerkeeper.HeaderKeeper;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.slamra.Slamra;

import java.util.function.Supplier;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;
import om.self.ezftc.utils.VectorMath;
import om.self.supplier.suppliers.EdgeSupplier;
import om.self.task.core.Task;
import om.self.task.core.TaskEx;

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
@TeleOp(name="3 odo bot test w/ relative move", group="Linear Opmode")
public class Test extends LinearOpMode {
    @Override
    public void runOpMode() {

        Robot r = new Robot(this);
        Drive d = new Drive(r);
        new DriveTeleop(d);

        PositionTracker pt = new PositionTracker(r);
//        new EncoderTracker(pt);
        new Odometry(pt);
        //new Slamra(pt);
        //PositionSolver ps = new PositionSolver(d, PositionSolverSettings.makeDefaultWithoutAlwaysRun());
        XRelativeSolver solver = new XRelativeSolver(d);

        Supplier<Boolean> moveForward = new EdgeSupplier(() -> gamepad1.dpad_up).getRisingEdgeSupplier();
        Supplier<Boolean> moveRight = new EdgeSupplier(() -> gamepad1.dpad_right).getRisingEdgeSupplier();

        r.init();

        waitForStart();

        r.start();

        pt.positionSourceId = Odometry.class;

        while (opModeIsActive()) {
            r.run();
            r.opMode.telemetry.addData("position", pt.getCurrentPosition());
            r.opMode.telemetry.addData("tile position", VectorMath.divide(pt.getCurrentPosition(), 23.5));
            if(r.opMode.gamepad1.a) r.opMode.telemetry.addData("task manager", r.getTaskManager());
            if(r.opMode.gamepad1.b) r.opMode.telemetry.addData("event manager", r.getEventManager());
            if(gamepad1.y) solver.setNewTarget(10, true);

//            if(moveForward.get())
//                ps.setNewTarget(pt.getCurrentPosition().addY(5), true);
//            if(moveRight.get())
//                ps.setNewTarget(pt.getCurrentPosition().addX(5), true);

            r.opMode.telemetry.update();
        }

        r.stop();
    }
}
