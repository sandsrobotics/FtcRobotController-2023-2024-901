package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.apriltags.Tag;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.lifter.Lifter;
import org.firstinspires.ftc.teamcode.parts.lifter.LifterControl;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.slamra.Slamra;

import java.text.DecimalFormat;
import java.util.function.Function;

import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.core.TaskEx;
import om.self.task.other.TimedTask;
@Disabled
@Autonomous(name="SLAMRA-auto-RIGHT-DANGEROUS", group="Test")
public class TestAutoSLAMRA extends LinearOpMode{
    public Function<Vector3, Vector3> transformFunc;
    public boolean isRight;
    public int targetPole;
    public boolean parkOnly;
    public Vector3 customStartPos;
    public boolean shutdownps;

    Lifter l;
    PositionSolver positionSolver;
    PositionTracker pt;
    Tag aprilTag;
    Vector3 pushSignal = new Vector3(-1.5, 0.5, 90);
    EncoderTracker et;
    public void initAuto(){
        transformFunc = (v) -> v;
        isRight = true;
        targetPole = 3;
    }

    private Vector3 tileToInchAuto(Vector3 tiles){
        return Constants.tileToInch(transformFunc.apply(tiles));
    }

    @Override
    public void runOpMode() {
        initAuto();

        Robot r = new Robot(this);
        Drive d = new Drive(r);

        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), new Vector3(-1.5 * Constants.tileSide,62,90));
//        pts.withPosition(transformFunc.apply(pts.startPosition));
        pts = pts.withPosition(customStartPos != null ? customStartPos : transformFunc.apply(pts.startPosition));
        pt = new PositionTracker(r, pts, PositionTrackerHardware.makeDefault(r));

        Slamra s = new Slamra(pt);
        //new Odometry(pt);
        l = new Lifter(r);
        positionSolver = new PositionSolver(d);
        aprilTag = new Tag(r);
        et = new EncoderTracker(pt);

        DecimalFormat df = new DecimalFormat("#0.0");
        r.init();
        s.updateSlamraPosition();
        Vector3 startupPose = s.slamraRawPose;
        while (!isStarted()) {
            s.updateSlamraPosition();
            aprilTag.onRun();
            telemetry.addData("raw position", s.slamraRawPose);
            if(!startupPose.equals(s.slamraRawPose)) telemetry.addLine("***** SLAMRA READY *****");
            telemetry.addLine(String.format("\nDetected tag ID=%s", aprilTag.detectedID));
            telemetry.addLine(String.format("\nPark ID=%s", aprilTag.parkID));
            r.opMode.telemetry.update();
            sleep(50);
        }
        r.start();
        if(shutdownps)
            positionSolver.triggerEvent(Robot.Events.STOP);

        Group container = new Group("container", r.taskManager);
        TimedTask autoTask = new TimedTask("auto task", container);
        TimedTask killer = new TimedTask("killer", container);
        positionSolver.setNewTarget(pt.getCurrentPosition(), true);
        TaskEx posTrackSwitch = new TaskEx("pos track switch", container);
////        // AFTER 27 SECONDS, PARK!
//        killer.addDelay(25000);
//        killer.addStep(() -> autoTask.runCommand(Group.Command.PAUSE));
//        park(killer);




        /******** Autonomous Setup ****************
         start position (-1.5,2.68,90)
         Various autonomous paths setup at bottom
         ******************************************/
        //IMPORTANT!! Need to go to parking safe position at the end of all tasks

        //posTrackSwitch.addStep(s::isSlamraDead);
//        posTrackSwitch.addStep(() -> pt.positionSourceId = EncoderTracker.class);
        l.setCone(4);
        autoTask.addStep(() -> {
            LifterControl.flipOpen = 0;
            l.setGrabberClosed();
        });
        if(!shutdownps) {
            if (!parkOnly) {
                switch (targetPole) {
                    case 3:
                        setupSideDangerous(autoTask);
                        break;
                    case 2:
                        moveSideTall(autoTask);
                        break;
                    case 1:
                        moveSideMid(autoTask);
                        break;
                    default:
                        throw new RuntimeException("pick a pole idiot!");
                }
            }
            if (parkOnly) parkOnly(autoTask);
            else park(autoTask);
        }
        while (opModeIsActive()) {
            telemetry.addData("pos id", pt.positionSourceId);

            r.run();
            //container.run();
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("Ultra [Left : Mid : Right]", "["
                    + df.format(l.getLeftUltra()) + " : "
                    + df.format(l.getMidUltra()) + " : "
                    + df.format(l.getRightUltra()) +"]");
            if(gamepad1.dpad_down) telemetry.addData("tasks", r.getTaskManager());
            if(gamepad1.dpad_down) telemetry.addData("events", r.getEventManager());
            telemetry.update();
        }
        r.stop();
    }

    private void park(TaskEx autoTask){
        //Parking locations
        Vector3 locOne = isRight ? new Vector3(-0.5, .5, 180) : new Vector3(-2.5, .5, 180);
        Vector3 locTwo = new Vector3(-1.5, .5, 90);
        Vector3 locThree = isRight ? new Vector3(-2.5, .5, 180) : new Vector3(-0.5, .5, 180);
        int parkId = aprilTag.parkID;
        //Parking task
        positionSolver.addMoveToTaskEx(tileToInchAuto(new Vector3(-1.5, .5, 90)), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(parkId == 1 ? locOne : parkId == 2 ? locTwo : locThree), autoTask);
    }

    private void parkOnly(TimedTask autoTask){

        Vector3 locOne = isRight ? new Vector3(-0.5, 1.5, 90) : new Vector3(-2.5, 1.5, 90);
        Vector3 locTwo = new Vector3(-1.5, 1.5, 90);
        Vector3 locThree = isRight ? new Vector3(-2.5, 1.5, 90) : new Vector3(-0.5, 1.5, 90);
        int parkId = aprilTag.parkID;
        //Parking task
//        autoTask.addDelay(15000);
        positionSolver.addMoveToTaskEx(tileToInchAuto(new Vector3(-1.5, 1.5, 90)), autoTask);
        if(!(parkId == 2))
            positionSolver.addMoveToTaskEx(tileToInchAuto(parkId == 1 ? locOne : locThree), autoTask);
        autoTask.addStep(() -> positionSolver.triggerEvent(Robot.Events.STOP));
    }

    private  void autoCircuit(TimedTask autoTask){
        Vector3 terminal = new Vector3(-2.5,2.6,180);
        Vector3 preTerm = new Vector3(-2.3,0.5,90);
        Vector3 preCPL = new Vector3();
        Vector3 closePoleLow = new Vector3();
        Vector3 stack = new Vector3();
        Vector3 poleMid = new Vector3();
    }

    private void moveSideMid(TimedTask autoTask){
        Vector3 rightLoadedPrep = new Vector3(-40.89 / 23.5,11.45 / 23.5,225);
        Vector3 specificPoleM = new Vector3(-1.2175,0.7875,225);
        Vector3 rightTallPrep = new Vector3(-40.89 / 23.5,11.45 / 23.5,180);
        Vector3 rightStack = new Vector3(-60.41 / 23.5,13.10 / 23.5,180);
        Vector3 preloadMid = new Vector3(-1.5,0.5,225);
        Vector3 poleM = new Vector3(-29.4 / 23.5, 17.74 / 23.5, 225);


        autoTask.addStep(()-> positionSolver.setMaxPower(.77,.77,.77));
        positionSolver.addMoveToTaskEx(tileToInchAuto(pushSignal), autoTask);
        l.addAutoPreDropToTask(autoTask, 2, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(preloadMid), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(poleM), autoTask);
        //autoTask.addDelay(1000);
        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);
        //autoTask.addDelay(100);

//        autoTask.addStep(() -> l.setLiftPosition(l.getLiftPosition() - 250));
//        autoTask.addStep(l::isLiftInTolerance);

        l.addAutoDropToTask(autoTask);
        autoTask.addDelay(350); // was 1000
        l.addAutoDockToTask(autoTask);

        positionSolver.addMoveToTaskEx(tileToInchAuto(rightLoadedPrep), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
//        autoTask.addStep(()-> positionSolver.setMaxPower(.7,.7,.7));
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightStack), autoTask);
//        autoTask.addStep(()-> positionSolver.setMaxPower(1,1,1));

        l.addAutoMoveToConeToTaskEx(autoTask);
        l.addAutoGrabToTask(autoTask);
        autoTask.addDelay(800); // was 1000

        setupSideMid(autoTask);
        setupSideMid(autoTask);
        setupSideMid(autoTask);
        setupSideMid(autoTask);

        l.addAutoPreDropToTask(autoTask, 2, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
//        autoTask.addStep(()-> positionSolver.setMaxPower(.7,.7,.7));

        positionSolver.addMoveToTaskEx(tileToInchAuto(poleM), autoTask);
//        autoTask.addStep(()-> positionSolver.setMaxPower(1,1,1));

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

        l.addAutoDropToTask(autoTask);
        autoTask.addDelay(300); // was 1000
        l.addAutoDockToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
    }

    private void setupSideMid(TimedTask autoTask){
        // alliance side mid pole
        //start
        Vector3 rightTallPrep = new Vector3(-40.89 / 23.5,11.45 / 23.5,180);
        Vector3 rightStack = new Vector3(-60.41 / 23.5,13.10 / 23.5,180);
        Vector3 rightStackBackup = new Vector3(-2.57, .5, 180);
        Vector3 rightLoadedPrep = new Vector3(-40.89 / 23.5,11.45 / 23.5,225);
        Vector3 specificPoleM = new Vector3(-1.2175,0.7875,225);
//        Vector3 poleM = new Vector3(-1.25,0.77,225);
        Vector3 poleM = new Vector3(-29.32 / 23.5, 17.74 / 23.5, 225);

        l.addAutoPreDropToTask(autoTask, 2, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
//        positionSolver.addMoveToTaskEx(tileToInchAuto(rightLoadedPrep), autoTask);
//        autoTask.addStep(()-> positionSolver.setMaxPower(.7,.7,.7));

        positionSolver.addMoveToTaskEx(tileToInchAuto(poleM), autoTask);
//        autoTask.addStep(()-> positionSolver.setMaxPower(1,1,1));

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

        l.addAutoDropToTask(autoTask);
        autoTask.addDelay(750); // was 1000
        l.addAutoDockToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightLoadedPrep), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
//        autoTask.addStep(()-> positionSolver.setMaxPower(.7,.7,.7));
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightStack), autoTask);
//        autoTask.addStep(()-> positionSolver.setMaxPower(1,1,1));
        l.addAutoMoveToConeToTaskEx(autoTask);
        l.addAutoGrabToTask(autoTask);
        autoTask.addDelay(800); // was 1000
    }


    private void moveSideTall(TimedTask autoTask){
        Vector3 start = new Vector3(-1.5,2.68,90);
        Vector3 through1T = new Vector3(-0.5,2.5,90);
        Vector3 through2T = new Vector3(-0.5,2,90);
        Vector3 preloadDepositT = new Vector3(-0.55,1.25,135);
        Vector3 through3T = new Vector3(-0.5,1.5,135);
        Vector3 poleStopT = new Vector3(-0.5,0.5,180);
        Vector3 wallStop = new Vector3(-1.5,0.5,180);
        Vector3 wall = new Vector3(-2.68,0.5,180);
        Vector3 poleT = new Vector3(-0.2175,0.7875,225);

        positionSolver.addMoveToTaskEx(tileToInchAuto(through1T), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(through2T), autoTask);
        l.addAutoPreDropToTask(autoTask, 3, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(preloadDepositT), autoTask);

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

        l.addAutoDropToTask(autoTask);
        //dock while moving
        positionSolver.addMoveToTaskEx(tileToInchAuto(through3T), autoTask);



        setupSideTall(autoTask);
        setupSideTall(autoTask);
        setupSideTall(autoTask);
        setupSideTall(autoTask);
        setupSideTall(autoTask);

        l.addAutoDockToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(poleStopT), autoTask);

    }

    private void setupSideTall(TimedTask autoTask){
        // alliance side tall pole
        Vector3 start = new Vector3(-1.5,2.68,90);
        Vector3 through1T = new Vector3(-0.5,2.5,90);
        Vector3 through2T = new Vector3(-0.5,2,90);
        Vector3 preloadDepositT = new Vector3(-0.55,1.25,135);
        Vector3 through3T = new Vector3(-0.5,1.5,135);
        Vector3 poleStopT = new Vector3(-0.5,0.5,180);
        Vector3 wallStop = new Vector3(-1.5,0.5,180);
        Vector3 wall = new Vector3(-2.68,0.5,180);
        Vector3 poleT = new Vector3(-0.2175,0.7875,225);

//        // alliance side tall pole
//        positionSolver.addMoveToTaskEx(tileToInchAuto(through1T), autoTask);
//        positionSolver.addMoveToTaskEx(tileToInchAuto(through2T), autoTask);
//        l.addAutoPreDropToTask(autoTask, 3, false);
//        positionSolver.addMoveToTaskEx(tileToInchAuto(preloadDepositT), autoTask);
//        l.addAutoDropToTask(autoTask);
//        //dock while moving
//        positionSolver.addMoveToTaskEx(tileToInchAuto(through3T), autoTask);
        // REPEATABLE CYCLE
        l.addAutoDockToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(poleStopT), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(wallStop), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(wall), autoTask);
        l.addAutoMoveToConeToTaskEx(autoTask);
        //grab
        l.addAutoGrabToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(poleStopT), autoTask);
        l.addAutoPreDropToTask(autoTask, 3, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(poleT), autoTask);

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

        l.addAutoDropToTask(autoTask);
        // END REPEATABLE CYCLE
    }




    private void setupSideDangerous(TimedTask autoTask){
        // dangerous tall pole
        Vector3 rightLoadedPrep = new Vector3(-1.5,.5,90);
        Vector3 afterStackPrep = new Vector3(-1.5, .5, 180);
        Vector3 rightTallPrep = new Vector3(-1.5,0.5,180);
        Vector3 rightTall = new Vector3(-1.21, 0.21, 135);
        Vector3 rightStack = new Vector3(-2.68,.5,180);

        //  dangerous tall pole path
        positionSolver.addMoveToTaskEx(tileToInchAuto(pushSignal), autoTask);

        l.addAutoPreDropToTask(autoTask, 3, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightLoadedPrep), autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTall), autoTask);

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

        //        positionSolver.triggerEvent(Robot.Events.STOP);
        l.addAutoDropToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
        l.addAutoDockToTask(autoTask);
        autoTask.addDelay(500);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightStack), autoTask);
        autoTask.addDelay(500);
        //Not working for some reason (not moving?):   l.addAutoMoveToConeToTaskEx(autoTask);
        l.addAutoGrabToTask(autoTask);
        autoTask.addDelay(1000);
        positionSolver.addMoveToTaskEx(tileToInchAuto(afterStackPrep),autoTask);
        l.addAutoPreDropToTask(autoTask, 3, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTall), autoTask);

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

//        positionSolver.triggerEvent(Robot.Events.STOP);
        l.addAutoDropToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
        l.addAutoDockToTask(autoTask);
        autoTask.addDelay(500);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightStack), autoTask);
//        l.addAutoMoveToConeToTaskEx(autoTask);
        l.addAutoGrabToTask(autoTask);
        autoTask.addDelay(1000);
        positionSolver.addMoveToTaskEx(tileToInchAuto(afterStackPrep),autoTask);
        l.addAutoPreDropToTask(autoTask, 3, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTall), autoTask);

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

        //        positionSolver.triggerEvent(Robot.Events.STOP);
        l.addAutoDropToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
        l.addAutoDockToTask(autoTask);
        autoTask.addDelay(500);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightStack), autoTask);
//        l.addAutoMoveToConeToTaskEx(autoTask);
        l.addAutoGrabToTask(autoTask);
        autoTask.addDelay(1000);
        positionSolver.addMoveToTaskEx(tileToInchAuto(afterStackPrep),autoTask);
        l.addAutoPreDropToTask(autoTask, 3, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTall), autoTask);

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

        //        positionSolver.triggerEvent(Robot.Events.STOP);
        l.addAutoDropToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
        l.addAutoDockToTask(autoTask);
        autoTask.addDelay(500);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightStack), autoTask);
//        l.addAutoMoveToConeToTaskEx(autoTask);
        l.addAutoGrabToTask(autoTask);
        autoTask.addDelay(1000);
        positionSolver.addMoveToTaskEx(tileToInchAuto(afterStackPrep),autoTask);
        l.addAutoPreDropToTask(autoTask, 3, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTall), autoTask);

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

        //        positionSolver.triggerEvent(Robot.Events.STOP);
        l.addAutoDropToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
        l.addAutoDockToTask(autoTask);
        autoTask.addDelay(500);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightStack), autoTask);
//        l.addAutoMoveToConeToTaskEx(autoTask);
        l.addAutoGrabToTask(autoTask);
        autoTask.addDelay(1000);
        positionSolver.addMoveToTaskEx(tileToInchAuto(afterStackPrep),autoTask);
        l.addAutoPreDropToTask(autoTask, 3, false);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTall), autoTask);

        autoTask.addTimedStep(() -> {}, l::isPoleInRange, 3000);

//        positionSolver.triggerEvent(Robot.Events.STOP);
        l.addAutoDropToTask(autoTask);
        positionSolver.addMoveToTaskEx(tileToInchAuto(rightTallPrep), autoTask);
        l.addAutoDockToTask(autoTask);
        autoTask.addDelay(500);
        //pull away from stack after lifiting!!!
    }
}


/***************************************************************************************************
 * Old Telemetry from active OPMODE
 *             //telemetry.addData("raw position", s.slamraRawPose);
 *             //telemetry.addData("tilt position", l.getCurrentTurnPosition());
 *             //telemetry.addData("is closed", l.isGrabberClosed());
 *             //telemetry.addData("right servo offset", l.getSettings().rightTurnServoOffset);
 *             //telemetry.addData("lift position:",df.format(l.getLiftPosition()));
 *
 *                         //condition ? run if true : condition ? run if true : condition ? run if true : throw new RuntimeError("im dead :(");
 *
 * //            if(gamepad2.dpad_left){
 * //                l.getSettings().rightTurnServoOffset -= 0.0001;
 * //            }
 * //            if(gamepad2.dpad_right){
 * //                l.getSettings().rightTurnServoOffset += 0.0001;
 * //            }
 */