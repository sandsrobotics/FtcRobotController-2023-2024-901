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
import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.SolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry24;
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
@Autonomous(name="Auto-Red-wall-and-ALL", group="Test")
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
    //DriveControl control;
    Led leds;
    public boolean center, left;
    public boolean midPark;
    public boolean isRed;
    public boolean parkOnly;
    public boolean isBoard;

    //Vector3 centralspikemark = new Vector3(-35.25, -39.5, -90);
   // Vector3 startPosition = new Vector3(-1.5,-2.7,-90);
    Vector3 startPosition = new Vector3(-1.5 * 23.5,-62,-90);

    public void initAuto(){
        transformFunc = (v) -> v;
        isRed = true;
        midPark = true;
        parkOnly = false;
        isBoard = false;
    }

    private Vector3 tileToInchAuto(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles));
    }

    private Vector3 tileToInchAutoNoZ(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles)).withZ(tiles.Z);
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
        leds = new Led(r);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), startPosition);
//            Vector3 startPosition = new Vector3(-1.5 * 23.5,-62,-90); // red wall
//        customStartPos = new Vector3(.5 * 23.5,-62,-90); // red board side
//         customStartPos = new Vector3(-1.5 * 23.5,-62,90); // blue wall side

        pts = pts.withPosition(customStartPos != null ? customStartPos : transformFunc.apply(pts.startPosition));
        pt = new PositionTracker(r, pts, PositionTrackerHardware.makeDefault(r));

//        XRelativeSolver solver = new XRelativeSolver(d);
        et = new EncoderTracker(pt);
////        Odometry odo = new Odometry(pt);
//        Odometry24 odo = new Odometry24(pt);
        pt.positionSourceId = EncoderTracker.class;

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
//        aprilTag.setDesiredTag(6); // FOR TESTING TAKE OUT AFTER

        Group container = new Group("container", r.taskManager);
        TimedTask autoTask = new TimedTask("auto task", container);
        positionSolver.setNewTarget(pt.getCurrentPosition(), true);

        autoTask.addStep(() -> {
            // init intake setup items here
        });
        // add calls to special autonomous action collections in methods below

//        testRobot(autoTask);

        if(!parkOnly) {
            if (isBoard) testRobot(autoTask);
            else wallAuto(autoTask);
            parkAuto(autoTask);
        }


        while (opModeIsActive()) {
            telemetry.addData("Team Prop Position", tp.pipeline.position);
//            double x = pt.getCurrentPosition().X;
//            double y = pt.getCurrentPosition().Y;
//            double z = Math.toRadians(pt.getCurrentPosition().Z);
//
//            double x1 = Math.cos(z)*8;
//            double y1 = Math.sin(z)*8;
//            packet.fieldOverlay().setFill("blue").fillCircle(x,y,6);
//            packet.fieldOverlay().setStroke("red").strokeLine(x,y,x+x1,y+y1);
//            packet.fieldOverlay().fillCircle(1, 1, 1);

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

//            dashboardTelemetry.update();
            telemetry.update();

        }
        r.stop();
    }

    private void parkAuto(TimedTask autoTask){
        Vector3 parkMid = new Vector3(2, -.5, 180);
        Vector3 parkSide = new Vector3(2, -2.5, 180);

        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskEx(tileToInchAuto(midPark ? parkMid : parkSide), autoTask);
    }

    private void wallAuto(TimedTask autoTask) {
        Vector3 startPos = new Vector3(-1.5, -2.6, -90);
        Vector3 pushProp = new Vector3(-1.5, -1.5, -90);
        Vector3 dropCenter = new Vector3(-1.5, -.6, -90);
        Vector3 dropPixCenter = new Vector3(-1.5, -1.5, -90);
        Vector3 dropPixLeft = new Vector3(-1.5, -1.38, 180);
        Vector3 dropPixRight = new Vector3(-1.5, -1.38, 0);
        Vector3 dPLBlue = new Vector3(-1.5, -1.38, 0);
        Vector3 dPRBlue = new Vector3(-1.5, -1.38, 180);
        Vector3 tagAngle = new Vector3(-1.5, -1.5, 180);
        Vector3 centerTagAngle = new Vector3(-1.5, .5, 180);
        Vector3 preSetupTagsMid = new Vector3(-1.5, -.5, 180); // tjk not over center
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180); // tjk not over center
        Vector3 preSetupTagsWall = new Vector3(-1.5, -2.5, 180);
        Vector3 setupTagsWall = new Vector3(1.5, -2.5, 180);
        Vector3 goToTags = new Vector3(1.5, -1.5, 180);
        Vector3 goCloseToTags = new Vector3(1.9, -1.5, 180); // tjk closer to board for testing
        Vector3 prePark = new Vector3(1.5, -1.5, 180);
        Vector3 centerAT = new Vector3(1.5,-1.5,180);
        Vector3 leftAT = new Vector3(1.5, -1.3, 180);
        Vector3 rightAT = new Vector3(1.5, -1.7, 180);

        positionSolver.setSettings(PositionSolverSettings.defaultSettings);
        if (center){
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropCenter), autoTask);
        }
        else {
            positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
            if(isRed)
                positionSolver.addMoveToTaskEx(tileToInchAuto(left ? dropPixLeft : dropPixRight), autoTask);
            else
                positionSolver.addMoveToTaskEx(tileToInchAuto(left ? dPLBlue : dPRBlue), autoTask);
        }
        intake.addAutoGrabToTask(autoTask, true, 2000); // make work laterr
        autoTask.addDelay(1000);
        positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsMid), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);
//        positionSolver.addMoveToTaskEx(tileToInchAuto(goToTags), autoTask);
//        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));
        if(center)
            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(centerAT), autoTask);
        else if(left)
            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(leftAT), autoTask);
        else
            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(rightAT), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));
        autoTask.addDelay(5000);
        intake.addFoundRangeToTask(autoTask);
        autoTask.addDelay(5000);
        intake.addAutoDropToTask(autoTask);
        autoTask.addDelay(5000);
        intake.addFinishDropToTask(autoTask);
        autoTask.addDelay(5000);
        intake.addAutoDockToTask(autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskEx(tileToInchAuto(prePark), autoTask);
    }

    private void boardAuto(TimedTask autoTask){
        Vector3 startPos = new Vector3(.5, -2.6, 90);
        Vector3 pushProp = new Vector3(.5, 0, 90);
        Vector3 preDrop = new Vector3(.5, -1.5, -90);
        Vector3 dropPixCenter = new Vector3(.5, -1.5, 90);
        Vector3 dropPixLeft = new Vector3(.5, -1.5, 180);
        Vector3 dropPixRight = new Vector3(1.5, -1.5, 0);
        Vector3 tagAngle = new Vector3(.5, -1.5, 0);
        Vector3 setupTags = new Vector3(1.5, -1.5, 0);
        Vector3 prePark = new Vector3(1.5, -2.5, 0);
        Vector3 goCloseToTags = new Vector3(1.8, -1.5, 180);
        Vector3 centerAT = new Vector3(1.8,-1.5,0);
        Vector3 leftAT = new Vector3(1.8, -1.3, 0);
        Vector3 rightAT = new Vector3(1.8, -2, 0);

        positionSolver.setSettings(PositionSolverSettings.defaultSettings);
        //if its in the center we need to push the prop out of the way and then place it or place it on the right side. if its on the left side normal place and back up to april tags, if its on the right side then back up first then place it on the thing, but also need to get rid of team prop somehow
        positionSolver.addMoveToTaskEx(tileToInchAuto(preDrop), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(center ? dropPixCenter : left ? dropPixLeft : dropPixRight), autoTask);
        intake.addAutoGrabToTask(autoTask, true, 2000);
        positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings); // let ranging take over
        if(!center && !left)
            positionSolver.addMoveToTaskEx(tileToInchAuto(tagAngle), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(setupTags), autoTask);
        if(center)
            positionSolver.addMoveToTaskEx(tileToInchAuto(centerAT), autoTask);
        else if(left)
            positionSolver.addMoveToTaskEx(tileToInchAuto(leftAT), autoTask);
        else
            positionSolver.addMoveToTaskEx(tileToInchAuto(rightAT), autoTask);
        autoTask.addDelay(10000);
        intake.addAutoDropToTask(autoTask);
        autoTask.addDelay(3000);
        intake.addFinishDropToTask(autoTask);
        autoTask.addDelay(1000);
        intake.addAutoDockToTask(autoTask);
        positionSolver.setSettings(PositionSolverSettings.defaultSettings);
        positionSolver.addMoveToTaskEx(tileToInchAuto(prePark), autoTask);

    }

    private void tjkRobot(TimedTask autoTask) {
        Vector3 goToTags = new Vector3(1.5, -1.5, 180);
        Vector3 goCloseToTags = new Vector3(1.9, -1.5, 180); // tjk closer to board for testing
        Vector3 prePark = new Vector3(1.5, -1.5, 180);
        Vector3 centerAT = new Vector3(1.8,-1.5,180);
        Vector3 leftAT = new Vector3(1.8, -1.3, 180);
        Vector3 rightAT = new Vector3(1.8, -1.7, 180);

        //positionSolver.addMoveToTaskEx(tileToInchAuto(goToTags), autoTask);
        if(center)
            positionSolver.addMoveToTaskEx(tileToInchAuto(centerAT), autoTask);
        else if(left)
            positionSolver.addMoveToTaskEx(tileToInchAuto(leftAT), autoTask);
        else
            positionSolver.addMoveToTaskEx(tileToInchAuto(rightAT), autoTask);
        autoTask.addDelay(3000);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));
        autoTask.addTimedStep(() -> {}, ()->intake.getBackDist() <= 8, 5000);
        intake.addAutoDropToTask(autoTask);
        autoTask.addDelay(2000);
        intake.addFinishDropToTask(autoTask);
        autoTask.addDelay(1000);
        intake.addAutoDockToTask(autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskEx(tileToInchAuto(prePark), autoTask);
    }
    private void testRobot(TimedTask autoTask){
        Vector3 startPos = new Vector3(1, 0, 90);
        Vector3 testPos = new Vector3(1, .5, 90);

        positionSolver.setSettings(PositionSolverSettings.defaultSettings);
        intake.addAutoGrabToTask(autoTask, true, 2000);
        autoTask.addDelay(5000);
        positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings);
        positionSolver.addMoveToTaskEx(tileToInchAuto(new Vector3(-1.5, -2.6, -90)), autoTask);
        intake.addFoundRangeToTask(autoTask);
        autoTask.addDelay(5000);
        intake.addAutoDropToTask(autoTask);
        autoTask.addDelay(5000);
        intake.addFinishDropToTask(autoTask);
        autoTask.addDelay(5000);
        intake.addAutoDockToTask(autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskEx(tileToInchAuto(new Vector3(-1.5, -2.6, -90)), autoTask);
//        intake.addAutoDropToTask(autoTask);
//        autoTask.addDelay(5000);
//        intake.addFinishDropToTask(autoTask);
//        autoTask.addDelay(5000);
//        intake.addAutoDockToTask(autoTask);
//        autoTask.addDelay(3000);
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
