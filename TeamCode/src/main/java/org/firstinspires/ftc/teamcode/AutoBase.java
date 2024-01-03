package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamProp;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamPropDetectionPipeline;

import java.sql.Time;
import java.text.DecimalFormat;
import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.core.TaskEx;
import om.self.task.other.TimedTask;

import static om.self.ezftc.utils.Constants.tileSide;

@Config
@Autonomous(name="AutoBase", group="Test")
public class AutoBase extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public Vector3 customStartPos;
    public boolean shutdownps;
    Intake intake;
    PositionSolver positionSolver;
    PositionTracker pt;
    EncoderTracker et;
    public AprilTag aprilTag;
    TeamProp tp;
    DriveControl control;
    public boolean center, left;
    public boolean midPark;
    public boolean isRed;
    public boolean parkOnly;

    //Vector3 centralspikemark = new Vector3(-35.25, -39.5, -90);
    Vector3 startPosition = new Vector3(-1.5,-2.7,90);

    public void initAuto(){
        transformFunc = (v) -> v;
        isRed = true;
        midPark = true;
        parkOnly = false;
    }

    private Vector3 tileToInchAuto(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles));
    }

    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }

    @Override
    public void runOpMode() {
        initAuto();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Robot r = new Robot(this);
        Drive d = new Drive(r);
        tp = new TeamProp(r);
        intake = new Intake(r);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), tileToInchAuto(startPosition));
        //pts = pts.withPosition(customStartPos != null ? customStartPos : transformFunc.apply(pts.startPosition));
        pt = new PositionTracker(r, pts, PositionTrackerHardware.makeDefault(r));

        XRelativeSolver solver = new XRelativeSolver(d);
        EncoderTracker et = new EncoderTracker(pt);

        //intake = new Intake(r, aprilTag);
        //positionSolverLose = new PositionSolver(d,PositionSolverSettings.loseSettings);
        positionSolver = new PositionSolver(d);
        DecimalFormat df = new DecimalFormat("#0.0");
        r.init();

        while (!isStarted()) {
            telemetry.addData("Team Prop", tp.pipeline.position);
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
            sleep(50);
        }

        tp.onStop();
        aprilTag = new AprilTag(r);
        aprilTag.onInit();
        r.start();

        center = (tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.CENTER);
        left = (tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.LEFT);


        if(shutdownps)
            positionSolver.triggerEvent(Robot.Events.STOP);


        if(isRed)
            aprilTag.setDesiredTag(center ? 5 : left ? 4 : 6);
        else
            aprilTag.setDesiredTag(center ? 2 : left ? 1 : 3);

        Group container = new Group("container", r.taskManager);
        TimedTask autoTask = new TimedTask("auto task", container);
        positionSolver.setNewTarget(pt.getCurrentPosition(), true);

        autoTask.addStep(() -> {
            // init intake setup items here
        });
        // add calls to special autonomous action collections in methods below

        if(!parkOnly)
        testRobot(autoTask);
        // gotoTestPos(autoTask);
        // wallAuto(autoTask);
        // boardAuto(autoTask);


        //parkAuto(autoTask);

        while (opModeIsActive()) {
            telemetry.addData("Team Prop Position", tp.pipeline.position);
            double x = pt.getCurrentPosition().X;
            double y = pt.getCurrentPosition().Y;
            double z = Math.toRadians(pt.getCurrentPosition().Z);

            double x1 = Math.cos(z)*8;
            double y1 = Math.sin(z)*8;
            packet.fieldOverlay().setFill("blue").fillCircle(x,y,6);
            packet.fieldOverlay().setStroke("red").strokeLine(x,y,x+x1,y+y1);
            packet.fieldOverlay().fillCircle(1, 1, 1);

            telemetry.addData("pos id", pt.positionSourceId);

            r.run();
            //container.run();
            telemetry.addData("position", pt.getCurrentPosition());
            if(gamepad1.dpad_down) telemetry.addData("tasks", r.getTaskManager());
            if(gamepad1.dpad_down) telemetry.addData("events", r.getEventManager());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("target found?", aprilTag.targetFound);
            if (aprilTag.targetFound) {
                telemetry.addData("Found", "ID %d (%s)", aprilTag.desiredTag.id, aprilTag.desiredTag.metadata.name);
                telemetry.addData("Range",  "%5.1f inches", aprilTag.desiredTag.ftcPose.range);
                telemetry.addData("X", "%5.1f inches", aprilTag.desiredTag.ftcPose.x);
                telemetry.addData("Bearing","%3.0f degrees", aprilTag.desiredTag.ftcPose.bearing);
                telemetry.addData("Yaw","%3.0f degrees", aprilTag.desiredTag.ftcPose.yaw);
            } else {
                telemetry.addData("\n>","Drive using joysticks to find valid target\n");
            }

            dashboardTelemetry.update();
            telemetry.update();

        }
        r.stop();
    }

    private void parkAuto(TimedTask autoTask){
        Vector3 parkMid = new Vector3(2, -.5, 0);
        Vector3 parkSide = new Vector3(2, -2.5, 0);

        positionSolver.addMoveToTaskEx(tileToInchAuto(midPark ? parkMid : parkSide), autoTask);
    }

    private void wallAuto(TimedTask autoTask){
        Vector3 startPos = new Vector3(-1.5, -2.6, 90);
        Vector3 pushProp = new Vector3(-1.5, -1, 90);
        Vector3 dropPixCenter = new Vector3(-1.5, -1.5, 90);
        Vector3 dropPixLeft = new Vector3(-1.5, -1.5, 180);
        Vector3 dropPixRight = new Vector3(-1.5, -1.5, 0);
        Vector3 tagAngle = new Vector3(-1.5, -1.5, 0);
        Vector3 preSetupTagsMid = new Vector3(-1.5, .5, 0);
        Vector3 setupTagsMid = new Vector3(1.5, .5, 0);
        Vector3 preSetupTagsWall = new Vector3(-1.5, -2.5, 0);
        Vector3 setupTagsWall = new Vector3(1.5, -2.5, 0);
        Vector3 goToTags = new Vector3(1.5, -1.5, 0);
        Vector3 prePark = new Vector3(1.5, -1.5, 0);

        positionSolver.setSettings(PositionSolverSettings.defaultSettings);
        positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixCenter), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(center ? dropPixCenter : left ? dropPixLeft : dropPixRight), autoTask);
        intake.addAutoGrabToTask(autoTask, true);
        positionSolver.addMoveToTaskEx(tileToInchAuto(tagAngle), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsWall), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsWall), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(goToTags), autoTask);
        positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings);
        autoTask.addDelay(1000);
        intake.addAutoDropToTask(autoTask);
        autoTask.addDelay(3000);
        intake.addAutoDockToTask(autoTask);
        positionSolver.setSettings(PositionSolverSettings.defaultSettings);
        positionSolver.addMoveToTaskEx(tileToInchAuto(prePark), autoTask);

    }

    private void boardAuto(TimedTask autoTask){
        Vector3 startPos = new Vector3(.5, -2.6, 90);
        Vector3 pushProp = new Vector3(.5, 0, 90);
        Vector3 dropPixCenter = new Vector3(.5, -1.5, 90);
        Vector3 dropPixLeft = new Vector3(.5, -1.5, 180);
        Vector3 dropPixRight = new Vector3(.5, -1.5, 0);
        Vector3 tagAngle = new Vector3(.5, -1.5, 0);
        Vector3 setupTags = new Vector3(1.5, -2.5, 0);
        Vector3 prePark = new Vector3(1.5, -1.5, 0);

        positionSolver.setSettings(PositionSolverSettings.defaultSettings);
        //if its in the center we need to push the prop out of the way and then place it or place it on the right side. if its on the left side normal place and back up to april tags, if its on the right side then back up first then place it on the thing, but also need to get rid of team prop somehow
        positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(center ? dropPixCenter : left ? dropPixLeft : dropPixRight), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(tagAngle), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(setupTags), autoTask);
        positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings);
        autoTask.addDelay(1000);
        intake.addAutoDropToTask(autoTask);
        autoTask.addDelay(3000);
        intake.addAutoDockToTask(autoTask);
        positionSolver.setSettings(PositionSolverSettings.defaultSettings);
        positionSolver.addMoveToTaskEx(tileToInchAuto(prePark), autoTask);

    }

    private void testRobot(TimedTask autoTask){
        Vector3 startPos = new Vector3(1, 0, 90);
        Vector3 testPos = new Vector3(1, .5, 90);

        intake.addAutoDropToTask(autoTask);
        autoTask.addDelay(3000);
        positionSolver.addMoveToTaskEx(tileToInchAuto(testPos), autoTask);
        autoTask.addDelay(3000);
        intake.addAutoDockToTask(autoTask);
        autoTask.addDelay(3000);
        intake.addAutoGrabToTask(autoTask, true);
        autoTask.addDelay(3000);
        intake.addAutoGrabToTask(autoTask, false);
    }

// ********* Put the methods that do the parts of the autonomous routines here ************//
    private void gotoTestPos(TimedTask autoTask) {
        //(-1.5,-2.6,90);
        Vector3 centralspikemark = new Vector3(-1.5, 0, 90);
        Vector3 pos1 = new Vector3(-1.5, 0, 90);
        Vector3 pos2 = new Vector3(-1.5, 0, 90);
        Vector3 pos3 = new Vector3(0.5,0,0);
        Vector3 centerAT = new Vector3(1.85,-1.5,0);
        Vector3 leftAT = new Vector3(1.85, -1.3, 0);
        Vector3 rightAT = new Vector3(1.85, -2, 0);

        positionSolver.setSettings(PositionSolverSettings.loseSettings);
        positionSolver.addMoveToTaskEx(tileToInchAuto(centralspikemark), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(pos1), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(pos2), autoTask);
       positionSolver.addMoveToTaskEx(tileToInchAuto(pos3), autoTask);
       positionSolver.setSettings(PositionSolverSettings.defaultSettings);
        if(tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.CENTER)
            positionSolver.addMoveToTaskEx(tileToInchAuto(centerAT), autoTask);
        else if(tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.LEFT)
            positionSolver.addMoveToTaskEx(tileToInchAuto(leftAT), autoTask);
        else
            positionSolver.addMoveToTaskEx(tileToInchAuto(rightAT), autoTask);
        positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings);
       // intake.addAutoDropToTask(autoTask);
    }
}
