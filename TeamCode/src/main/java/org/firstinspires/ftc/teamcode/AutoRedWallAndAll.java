package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.apriltag.AprilTag;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.intake.Intake;
import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry24;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamProp;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamPropDetectionPipeline;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import java.text.DecimalFormat;
import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.supplier.suppliers.EdgeSupplier;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

import static om.self.ezftc.utils.Constants.tileSide;

@Config
@Autonomous(name="1 RED-WALL", group="Test")
public class AutoRedWallAndAll extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public Vector3 customStartPos;
    public boolean shutdownps;
    Intake intake;
    PositionSolver positionSolver;
    PositionTracker pt;
    EncoderTracker et;
    public AprilTag aprilTag;
    TeamProp tp;
    Led leds;
    public boolean center, left, right;
    public boolean midPark;
    public boolean isRed;
    public boolean parkOnly;
    public boolean isBoard;
    public boolean extraPix;
    public boolean firstDrop;
    public boolean dropLow;
    static public int shortDelay = 1000;
    static public int midDelay = 2000;
    static public int longDelay = 3000;
    public int startDelay;
    private int parkPosition;
    private int pixels;
    //Vector3 centralspikemark = new Vector3(-35.25, -39.5, -90);
   // Vector3 startPosition = new Vector3(-1.5,-2.7,-90);
    Vector3 startPosition = new Vector3(-1.5 * 23.5,-62,-90);

    public void initAuto(){
        transformFunc = (v) -> v;
        isRed = true;
        midPark = true;
        parkOnly = false;
        isBoard = false;
        extraPix = true;
        dropLow = false;
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
        long start;
        initAuto();
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        Telemetry dashboardTelemetry = dashboard.getTelemetry();
        Robot r = new Robot(this);
        Drive d = new Drive(r);
        new BulkRead(r);
        tp = new TeamProp(r);
        intake = new Intake(r);
        leds = new Led(r);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), startPosition);
//            Vector3 startPosition = new Vector3(-1.5 * 23.5,-62,-90); // red wall
//        customStartPos = new Vector3(.5 * 23.5,-62,-90); // red board side
//         customStartPos = new Vector3(-1.5 * 23.5,-62,90); // blue wall side

        pts = pts.withPosition(customStartPos != null ? customStartPos : transformFunc.apply(pts.startPosition));
        pt = new PositionTracker(r, pts, PositionTrackerHardware.makeDefault(r));
        Odometry odo = new Odometry(pt);
        pt.positionSourceId = Odometry.class;
        positionSolver = new PositionSolver(d);
        DecimalFormat df = new DecimalFormat("#0.0");
        r.init();
        int maxDelay = isBoard ? 18000 : 6000;

        while (!isStarted()) {
            if (tp.pipeline.position== TeamPropDetectionPipeline.TeamPropPosition.NONE)
                telemetry.addData("***** WAIT *****\n***** WAIT *****\n***** WAIT *****", "");
            else
                telemetry.addData("Team Prop", tp.pipeline.position);

            if(new EdgeSupplier(()-> r.opMode.gamepad1.right_bumper).isRisingEdge()) {
                startDelay += 1000;
                if(startDelay > maxDelay) startDelay = maxDelay;
            }
            else if(new EdgeSupplier(()->r.opMode.gamepad1.left_bumper).isRisingEdge()) {
                startDelay -= 1000;
                if(startDelay < 0) startDelay = 0;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.a).isRisingEdge()) {
                parkPosition = 1;
                maxDelay = isBoard ? 18000 : 6000;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.b).isRisingEdge()) {
                parkPosition = 2;
                maxDelay = isBoard ? 18000 : 6000;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.x).isRisingEdge()) {
                parkPosition = 3;
                maxDelay = isBoard ? 20000 : 7000;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.y).isRisingEdge()) {
                parkPosition = 0;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.dpad_up).isRisingEdge()) {
                extraPix = true;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.dpad_down).isRisingEdge()) {
                extraPix = false;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.dpad_left).isRisingEdge()){
                dropLow = true;
            } else if(new EdgeSupplier(()->r.opMode.gamepad1.dpad_right).isRisingEdge()){
                dropLow = false;
            }


//            packet.put("Team Prop", tp.pipeline.position);
//            dashboard.sendTelemetryPacket(packet);
            telemetry.addData("EXTRA PIXEL? ", extraPix);
            telemetry.addData("DROP POSITION? ", dropLow ? "Drop first" : "Drop second");
            telemetry.addData("PARK POSITION", parkPosition == 0 ? "Park based off tags" : parkPosition == 1 ? "Park MID" : parkPosition == 2 ? "Park CORNER" : "Park BOARD");
            telemetry.addData("START DELAY", startDelay / 1000);
            telemetry.addData("AUTO: ", isRed ? (isBoard ? "RED BOARD" : "RED WALL") : isBoard ? "BLUE BOARD" : "BLUE WALL");
            telemetry.update();
            sleep(50);
        }

        tp.onStop();
        aprilTag = new AprilTag(r);
        aprilTag.onInit();
        r.start();

        center = (tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.CENTER);
        left = (tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.LEFT);
        right = (tp.pipeline.position == TeamPropDetectionPipeline.TeamPropPosition.RIGHT);

        if(shutdownps)
            positionSolver.triggerEvent(Robot.Events.STOP);

        if(isRed) {
            aprilTag.setDesiredTag(center ? 5 : left ? 4 : 6);
        }
        else {
            if(left)
                left = false;
            else if(right)
                left = true;
            aprilTag.setDesiredTag(center ? 2 : left ? 1 : 3);
        }

        Group container = new Group("container", r.taskManager);
        TimedTask autoTask = new TimedTask("auto task", container);
        positionSolver.setNewTarget(pt.getCurrentPosition(), true);

        autoTask.addStep(() -> {
            firstDrop = true;
            // init intake setup items here
        });
        // add calls to special autonomous action collections in methods below
        if(!parkOnly) {
            autoTask.addDelay(startDelay);
            if (isBoard) {
                if(extraPix)
                    boardAutoGrabPix(autoTask);
                else
                    boardAuto(autoTask);
            }
            else {
                if(extraPix)
                    wallAutoGrabPix(autoTask);
                else
                    wallAuto(autoTask);
            }
            dropAuto(autoTask);
            parkAuto(autoTask);
        } else
            testAuto(autoTask);


        while (opModeIsActive()) {
            start = System.currentTimeMillis();
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

            telemetry.addData("Pixles in bucket currently: ", pixels);
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
            r.opMode.telemetry.addData("time", System.currentTimeMillis() - start);
//            dashboardTelemetry.update();
            telemetry.update();

        }
        r.stop();
    }

    private void parkAuto(TimedTask autoTask){
        Vector3 parkMid = new Vector3(2.4, -.5, 180);
        Vector3 parkSide = new Vector3(2.4, -2.5, 180);
        Vector3 preParkMid = new Vector3(2.0, -.5, 180);
        Vector3 preParkSide = new Vector3(2.0, -2.5, 180);

        if(parkPosition == 0) {
            intake.addAutoDockToTask(autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(left ? preParkSide : preParkMid), autoTask);
            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(left ? parkSide : parkMid), autoTask);
        }
        else if(parkPosition == 3){
            positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings);
            intake.addAutoDockToTask(autoTask);
        }
        else{
            intake.addAutoDockToTask(autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(parkPosition == 1 ? preParkMid : preParkSide), autoTask);
            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(parkPosition == 1 ? parkMid : parkSide), autoTask);
        }
        autoTask.addStep(()->intake.setSweepPosition(1)); // for easier autonomous setup
    }

    private void dropAuto(TimedTask autoTask){
        Vector3 centerAT = new Vector3(1.5,-1.55,180);
        Vector3 leftAT = new Vector3(1.5, -1.23, 180);
        Vector3 rightAT = new Vector3(1.5, -1.82, 180);

        intake.addAutoDropToTask(autoTask);
        if(!isBoard)
            positionSolver.addMoveToTaskEx(tileToInchAuto(centerAT), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(center ? centerAT : left ? leftAT : rightAT), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));

//        if(!isBoard && extraPix){
            autoTask.addDelay(1500); //give swing arm time to get out before lowering slider
            autoTask.addStep(()-> intake.setSlidePosition(dropLow ? 650 : 1200));
            autoTask.addStep((Runnable) () -> intake.run = true);
            autoTask.addDelay(3000);
            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(center ? leftAT : centerAT), autoTask);
            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));
            autoTask.addStep(()-> intake.setSlidePosition(1200));
            autoTask.addDelay(shortDelay); //magic deelay *(DO NORT MREOVE)
//            autoTask.addStep((Runnable)()->firstDrop = false);
//        } else if(extraPix) {
//            autoTask.addStep(()-> intake.setSlidePosition(1200));
//            autoTask.addStep((Runnable) () -> intake.run = true);
//            autoTask.addDelay(1200);
//            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
//            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(center ? leftAT : centerAT), autoTask);
//            autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings));
//            autoTask.addDelay(shortDelay); //magic deelay *(DO NORT MREOVE)
//        } else{
//            autoTask.addDelay(1500); //give swing arm time to get out before lowering slider (ALSO MAGICAL)
//            autoTask.addStep((Runnable) ()->intake.completeDrop = false);
//            autoTask.addStep(()-> intake.setSlidePosition(dropLow ? 650 : 900));
//        }
        autoTask.addStep((Runnable) () -> intake.run = true);
        autoTask.addDelay(1500);
        autoTask.addStep(()->intake.setGrabPosition(1));
        autoTask.addDelay(3000); //make sure pixel is dropped before pulling away
        autoTask.addStep(()-> intake.setSlidePosition(!firstDrop ? 1200 : dropLow ? 750 : 1200));
        autoTask.addDelay(firstDrop ? 0 : 500); //give slider time to get up otherwise it wont dock properly and crash on park
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
//        positionSolver.addMoveToTaskEx(tileToInchAuto(pt.getCurrentPosition().withX(1.5*23.5)), autoTask); // Move away from the board so the swing arm doesn't catch on it
//        autoTask.addDelay(midDelay);


    }

    private void wallAuto(TimedTask autoTask) {
        Vector3 startPos = new Vector3(-1.5, -2.6, -90);
        Vector3 pushProp = new Vector3(-1.5, -1.38, -90);
        Vector3 centerRight = new Vector3(-1.7, -1.38, 0);
        Vector3 moveRight = new Vector3(-1.7, -.5, 0);
        Vector3 dropCenter = new Vector3(-1.5, -.6, -90);
        Vector3 dropPixLeft = new Vector3(-1.49, -1.38, 180);
        Vector3 postDropPixLeft = new Vector3(-1.5, -1.38, 180);
        Vector3 preDropPixRight = new Vector3(-1.47, -1.38, -90);
        Vector3 dropPixRight = new Vector3(-1.47, -1.38, 0);
        Vector3 preSetupTagsMid = new Vector3(-1.5, -.5, 180);
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);
        Vector3 preSetupTagsMidRIGHT = new Vector3(-1.7, -.5, 180);
        Vector3 stack = new Vector3(-2.5, -.5, 180);


        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTask.addStep(()->intake.setGrabPosition(3));
        if (center){
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropCenter), autoTask);
            intake.addAutoGrabToTask(autoTask, true);
            autoTask.addDelay(500);
            positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsMid), autoTask);

        }
        else if(left){
            positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixLeft), autoTask);
            intake.addAutoGrabToTask(autoTask, true);
            autoTask.addDelay(500);
            positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsMid), autoTask);
        } else {
            positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(preDropPixRight), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixRight), autoTask);
            intake.addAutoGrabToTask(autoTask, true);
            autoTask.addDelay(500);
            positionSolver.addMoveToTaskEx(tileToInchAuto(centerRight),autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(moveRight), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsMidRIGHT), autoTask);
        }
        positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);
    }

    private void boardAuto(TimedTask autoTask){
        Vector3 startPos = new Vector3(.5, -2.6, 90);
        Vector3 preDrop = new Vector3(.5, -1.38, -90);
        Vector3 dropPixCenter = new Vector3(.5, -.6, -90);
        Vector3 postCenter = new Vector3(1, 0, -90);
        Vector3 dropPixLeft = new Vector3(.5, -1.38, 180);
        Vector3 setupTags = new Vector3(1.5, -1.5, 180);
        Vector3 turnRight = new Vector3(1.37, -1.5, 180);

        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTask.addStep(()->intake.setGrabPosition(3));
        if(center){
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixCenter), autoTask);
            intake.addAutoGrabToTask(autoTask, true);
            autoTask.addDelay(500);
            positionSolver.addMoveToTaskExNoWait(tileToInchAuto(postCenter), autoTask);
        }
        else if (left) {
            positionSolver.addMoveToTaskEx(tileToInchAuto(preDrop), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixLeft), autoTask);
            intake.addAutoGrabToTask(autoTask, true);
        }
        else {
            positionSolver.addMoveToTaskEx(tileToInchAuto(turnRight), autoTask);
            intake.addAutoGrabToTask(autoTask, true);
        }
        autoTask.addDelay(250);
        positionSolver.addMoveToTaskEx(tileToInchAuto(setupTags), autoTask);
    }

    private void boardAutoGrabPix(TimedTask autoTask){
        Vector3 postTag = new Vector3(1, -2.5, 180);
        Vector3 throughRigging = new Vector3(-1.5, -2.5, 180);
        Vector3 preStack = new Vector3(-2.3, -1.5, 180);
        Vector3 preStackAvoidLeft = new Vector3(-2.3, -2, 180);
        Vector3 stack = new Vector3(-2.3, -1.5, 180);

        boardAuto(autoTask);
        dropAuto(autoTask);

        intake.addAutoDockToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(postTag), autoTask);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        positionSolver.addMoveToTaskEx(tileToInchAuto(throughRigging), autoTask);
//        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
        if(right)
           positionSolver.addMoveToTaskEx(tileToInchAuto(preStack), autoTask);
        else{
            positionSolver.addMoveToTaskEx(tileToInchAuto(preStackAvoidLeft), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(preStack), autoTask);
        }
        grabFromStackAuto(autoTask, 2, stack);

    }

    private void wallAutoGrabPix(TimedTask autoTask) {
    Vector3 startPos = new Vector3(-1.5, -2.6, -90);
    Vector3 pushProp = new Vector3(-1.5, -1.38, -90);
    Vector3 centerRight = new Vector3(-1.7, -1.38, 0);
    Vector3 moveRight = new Vector3(-1.7, -.5, 0);
    Vector3 dropCenter = new Vector3(-1.5, -.6, -90);
    Vector3 dropPixLeft = new Vector3(-1.47, -1.38, 180);
    Vector3 postDropPixLeft = new Vector3(-1.5, -1.38, 180);
    Vector3 preDropPixRight = new Vector3(-1.47, -1.38, -90);
    Vector3 dropPixRight = new Vector3(-1.47, -1.38, 0);
    Vector3 preSetupTagsMid = new Vector3(-1.5, -.5, 180);
    Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);
    Vector3 preSetupTagsMidRIGHT = new Vector3(-1.7, -.5, 180);
    Vector3 preStack = new Vector3(-2.2, -.52, 180);
    Vector3 stackRed = new Vector3(-2.4, -.5, 180);
    Vector3 stackBlue = new Vector3(-2.45, -.5, 180);
    Vector3 stackbackout = new Vector3(-2.2, -.52, 180); // tjk
    Vector3 directDropLeft = new Vector3(-2, -.5, -70);

        autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTask.addStep(() -> intake.setGrabPosition(1));
        if (center) {
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropCenter), autoTask);
            intake.addAutoGrabToTask(autoTask, true);
            autoTask.addDelay(250);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));

        } else if (left) {
            positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixLeft), autoTask);
            intake.addAutoGrabToTask(autoTask, true);
            autoTask.addDelay(250);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            if(!isRed)
                positionSolver.addMoveToTaskEx(tileToInchAuto(postDropPixLeft), autoTask);
        } else {
            positionSolver.addMoveToTaskEx(tileToInchAuto(pushProp), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(preDropPixRight), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(dropPixRight), autoTask);
            intake.addAutoGrabToTask(autoTask, true);
            autoTask.addDelay(250);
            autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.loseSettings));
            positionSolver.addMoveToTaskEx(tileToInchAuto(centerRight), autoTask);
            positionSolver.addMoveToTaskEx(tileToInchAuto(moveRight), autoTask);
        }
        positionSolver.addMoveToTaskEx(tileToInchAuto(preSetupTagsMid), autoTask);
        autoTask.addStep(() -> positionSolver.setSettings(PositionSolverSettings.defaultSettings));

        autoTask.addStep(() -> intake.setSweepPosition(3));
        positionSolver.addMoveToTaskEx(tileToInchAuto(preStack), autoTask);
        intake.addAutoGrabToTask(autoTask, false); // pickup from white stack
        positionSolver.addMoveToTaskEx(tileToInchAuto(isRed ? stackRed : stackBlue), autoTask);
        //intake.addAutoGrabToTask(autoTask, false); // pickup from white stack
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
        autoTask.addDelay(1000); // tjk
//        positionSolver.addMoveToTaskExNoWait(tileToInchAuto(preSetupTagsMid), autoTask);
//        intake.addAutoGrabToTask(autoTask, true);
        autoTask.addTimedStep(()->intake.sweepWithPower(1), 750); // dump any extra pixels
        autoTask.addStep(() -> intake.setGrabPosition(3));
//        autoTask.addDelay(500);
        autoTask.addStep(()-> setExtraPix(intake.hasPixels() == 2));
        autoTask.addStep((Runnable) ()-> intake.extraDrop = extraPix);
        positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);
    }

    public void grabFromStackAuto(TimedTask autoTask, int height, Vector3 stack){
        Vector3 postStack = new Vector3(-2.3, -.5, 180);
        Vector3 setupTagsMid = new Vector3(1.5, -.5, 180);

        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTask.addStep(() -> intake.setSweepPosition(height));
//        intake.addAutoGrabToTask(autoTask, false); // pickup from white stack
        positionSolver.addMoveToTaskEx(tileToInchAuto(stack), autoTask, 10000);
        autoTask.addTimedStep(()->intake.sweepWithPower(-1), ()->intake.hasPixels() == 2, 3000);
//        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.loseSettings));
        autoTask.addDelay(1000); // tjk
        autoTask.addTimedStep(()->intake.sweepWithPower(1), 750); // dump any extra pixels
        autoTask.addStep(() -> intake.setGrabPosition(3));
        autoTask.addStep(()-> setExtraPix(intake.hasPixels() == 2));
        autoTask.addStep((Runnable) ()-> intake.extraDrop = extraPix);
        if(isBoard)
            positionSolver.addMoveToTaskEx(tileToInchAuto(postStack), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(setupTagsMid), autoTask);

    }

    private void testAuto(TimedTask autoTask){
        Vector3 startPos = new Vector3(-1.4, -2.5, -90);
        Vector3 faceTag = new Vector3(-1.8, -2.5, 180);
        Vector3 faceTag2 = new Vector3(-1, -2.5, 180);
        Vector3 rightBoardAT = new Vector3(1.5, -1.75, 180);

        positionSolver.addMoveToTaskEx(tileToInchAuto(intake.hasPixels() == 2 ? faceTag : faceTag2), autoTask);
        autoTask.addDelay(3000);
//        autoTask.addStep(()->aprilTag.updatePositionWithTag());
        autoTask.addStep(() -> intake.setGrabPosition(3));
        autoTask.addDelay(3000);
        intake.addAutoDropToTask(autoTask);
//        positionSolver.addMoveToTaskEx(tileToInchAuto(rightBoardAT), autoTask);
        autoTask.addDelay(3000);
        intake.addFinishDropToTask(autoTask, extraPix);
        autoTask.addDelay(7000);
        intake.addAutoDockToTask(autoTask);
    }

    public void setExtraPix(boolean extraPix) {
        this.extraPix = extraPix;
    }
}
