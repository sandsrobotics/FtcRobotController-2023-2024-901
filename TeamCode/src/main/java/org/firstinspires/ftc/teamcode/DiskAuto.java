package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.diskfinder.DiskFinder;
import org.firstinspires.ftc.teamcode.parts.disklauncher.DiskLauncher;
import org.firstinspires.ftc.teamcode.parts.disklauncher.DiskLauncherTeleop;
import org.firstinspires.ftc.teamcode.parts.disklauncher.hardware.DiskLauncherHardware;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveTeleop;
import org.firstinspires.ftc.teamcode.parts.positionsolver.PositionSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.XRelativeSolver;
import org.firstinspires.ftc.teamcode.parts.positionsolver.settings.PositionSolverSettings;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.encodertracking.EncoderTracker;
import org.firstinspires.ftc.teamcode.parts.positiontracker.hardware.PositionTrackerHardware;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry;
import org.firstinspires.ftc.teamcode.parts.positiontracker.odometry.Odometry24;
import org.firstinspires.ftc.teamcode.parts.positiontracker.settings.PositionTrackerSettings;

import java.lang.reflect.Array;
import java.text.DecimalFormat;
import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

@Autonomous(name="Disk Demo", group="Linear Opmode")
public class DiskAuto extends LinearOpMode {
    double tileSide = 23.5;
    static public double rotatePower = .2;
    Drive drive;
    Robot robot;
    PositionSolver positionSolver;
    PositionTracker pt;
    DiskFinder diskFinder;

    public Vector3 tiletoField(Vector3 p){
        return new Vector3(p.X * tileSide, p.Y * tileSide, p.Z);
    }
    public Vector3 fieldToTile(Vector3 p){
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }

    Vector3 fieldStartPos = tiletoField(new Vector3(0,-1.5,0));

    public void initTeleop(){
        new DriveTeleop(this.drive);
    }

    @Override
    public void runOpMode() {
        DecimalFormat df = new DecimalFormat("#0.0");
        long start;
        FtcDashboard dashboard = FtcDashboard.getInstance();
        TelemetryPacket packet = new TelemetryPacket();
        robot = new Robot(this);
        drive = new Drive(robot);
        new BulkRead(robot);
        new DriveTeleop(this.drive);

        /***********Auto Stuff *****************/
        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2,2,2), fieldStartPos);
        pt = new PositionTracker(robot,pts,PositionTrackerHardware.makeDefault(robot));
        positionSolver = new PositionSolver(drive);
        positionSolver.setSettings(PositionSolverSettings.defaultNoAlwaysRunSettings);
        new EncoderTracker(pt);
        pt.positionSourceId = EncoderTracker.class;
        /*************************************/

        DiskLauncher launcher = new DiskLauncher(robot);
        new DiskLauncherTeleop(launcher);
        diskFinder = new DiskFinder(robot);
        robot.init();

        while (!isStarted()) {
            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }

        robot.start();
        //findDisks(diskAngles);
        /********** auto tasks *******/
        Group container = new Group("container", robot.taskManager);
        TimedTask autoTask = new TimedTask("auto task", container);

        //Vector3 startPos = new Vector3(0,-1.5*tileSide,0);
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.slowScanSettings));
        //Vector3 startPos = fieldStartPos;
        //positionSolver.addMoveToTaskEx(startPos, autoTask);
        Vector3 tenPts = tiletoField(new Vector3(1.25,-.3, 115));
        autoTask.addStep(()-> {
            positionSolver.setMaxPower(1, 1, rotatePower);
            DiskFinder.clearDiskPos();
        });
        positionSolver.addMoveToTaskExNoWait(fieldStartPos.withZ(89), autoTask);
        autoTask.addStep(()-> (positionSolver.isDone() || diskFinder.getDiskPos() != null));
        positionSolver.addMoveToTaskExNoWait(fieldStartPos.withZ(180), autoTask);
        autoTask.addStep(()-> (positionSolver.isDone() || diskFinder.getDiskPos() != null));
        autoTask.addStep(()-> {
            if (diskFinder.getDiskPos() != null) {
                positionSolver.setNewTarget(diskFinder.getDiskPos(), true);
            }
        });
        autoTask.addStep(()->positionSolver.setSettings(PositionSolverSettings.defaultSettings));
        autoTask.addDelay(1000);
        autoTask.addStep(()-> {
            if (diskFinder.getDiskPos() != null) {
                DiskLauncher.conveyerRunning = true;
                Vector3 getDisk = translateAlongZ(diskFinder.getDiskPos(), 17.0);
                positionSolver.setNewTarget(getDisk, true);
            }
        });
        autoTask.addDelay(2000);
        autoTask.addStep(()-> {
            if (diskFinder.getDiskPos() != null) {
                positionSolver.setNewTarget(tenPts, true);
            }
        });
        autoTask.addDelay(3000);
        autoTask.addStep(()-> {
            DiskLauncher.conveyerRunning = false;
            DiskLauncher.launcherRunning = true;
        });
        autoTask.addDelay(2000);
        autoTask.addStep(()->DiskLauncher.autoFlip = true);
        autoTask.addDelay(200);
        autoTask.addStep(()->DiskLauncher.autoFlip = false);
        autoTask.addDelay(500);
        autoTask.addStep(()->DiskLauncher.autoFlip = true);
        autoTask.addDelay(300);
        autoTask.addStep(()->DiskLauncher.autoFlip = false);
        autoTask.addDelay(500);
        autoTask.addStep(()-> DiskLauncher.launcherRunning = false);
        autoTask.addDelay(100);
        /*************************************/

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("relative position", pt.getRelativePosition());
            telemetry.addData("spinner RPM", launcher.getRPM());
            telemetry.addData("Disk Position", diskFinder.getDiskPos());
            telemetry.addData("loop time", System.currentTimeMillis() - start);
//            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        robot.stop();
    }

    public void moveRobot(Vector3 target){
        positionSolver.setNewTarget(target, false);
    }

    //****** tjk
    public void pixelShiftY(Double Y){
        positionSolver.setNewTarget(pt.getCurrentPosition().addY(Y), true);
    }

    public Vector3 translateAlongZ(Vector3 start, Double distance) {
        Double zRad = Math.toRadians(start.Z);
        Vector3 returnV = new Vector3(start.X,start.Y,start.Z);
        if (start != null) {

            returnV = returnV.addX(distance * Math.cos(zRad));
            returnV = returnV.addY(distance * Math.sin(zRad));
        }
        return returnV;
    }
}


