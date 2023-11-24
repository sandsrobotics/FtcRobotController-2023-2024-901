package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.intake.IntakeTeleop;

import java.text.DecimalFormat;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Vector3;

@TeleOp(name="Test Pixel", group="Linear Opmode")
public class TestPixel extends LinearOpMode {
    double tileSide = 23.5;
    boolean slideDone = false;
    public Vector3 tiletoField(Vector3 p){
        return new Vector3(p.X * tileSide, p.Y * tileSide, p.Z);
    }

    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }

    Vector3 fieldStartPos = new Vector3(-36,63,90);

    @Override
    public void runOpMode() {
        long start;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        Robot r = new Robot(this);
        Drive d = new Drive(r);
        new DriveTeleop(d);

        PositionTracker pt = new PositionTracker(r);
        //PositionSolver ps = new PositionSolver(d, PositionSolverSettings.makeDefaultWithoutAlwaysRun());
        XRelativeSolver solver = new XRelativeSolver(d);
        EncoderTracker et = new EncoderTracker(pt);
        //Odometry odo = new Odometry(pt);

        Intake i = new Intake(r);
        new IntakeTeleop(i);

        DecimalFormat df = new DecimalFormat("#0.0");

        r.init();

        while (!isStarted()) {}

        r.start();

        pt.positionSourceId = EncoderTracker.class;

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            r.run();

            double x = pt.getCurrentPosition().X;
            double y = pt.getCurrentPosition().Y;
            double z = Math.toRadians(pt.getCurrentPosition().Z);

            double x1 = Math.cos(z)*8;
            double y1 = Math.sin(z)*8;
            packet.fieldOverlay().setFill("blue").fillCircle(x,y,6);
            packet.fieldOverlay().setStroke("red").strokeLine(x,y,x+x1,y+y1);

            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("relative position", pt.getRelativePosition());

            r.opMode.telemetry.addData("time", System.currentTimeMillis() - start);

            if(gamepad1.dpad_down) {
                solver.setNewTarget(10, true);
            }

            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        r.stop();
    }
}
