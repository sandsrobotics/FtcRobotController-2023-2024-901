package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.parts.bulkread.BulkRead;
import org.firstinspires.ftc.teamcode.parts.diskfinder.DiskFinder;
import org.firstinspires.ftc.teamcode.parts.disklauncher.DiskLauncher;
import org.firstinspires.ftc.teamcode.parts.disklauncher.DiskLauncherTeleop;
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
import org.firstinspires.ftc.vision.apriltag.AprilTagGameDatabase;
import org.firstinspires.ftc.vision.apriltag.AprilTagLibrary;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.lang.reflect.Array;
import java.text.DecimalFormat;
import om.self.ezftc.core.Robot;
import om.self.ezftc.utils.Constants;
import om.self.ezftc.utils.Vector3;
import om.self.task.core.Group;
import om.self.task.other.TimedTask;

@TeleOp(name="Disk Launcher", group="Linear Opmode")
public class DiskDemo extends LinearOpMode {
    double tileSide = 23.5;
    static public double rotatePower = .2;
    Drive drive;
    Robot robot;
    PositionSolver positionSolver;
    PositionTracker pt;
    DiskFinder diskFinder;

    public Vector3 tiletoField(Vector3 p) {
        return new Vector3(p.X * tileSide, p.Y * tileSide, p.Z);
    }

    public Vector3 fieldToTile(Vector3 p) {
        return new Vector3(p.X / tileSide, p.Y / tileSide, p.Z);
    }

    Vector3 fieldStartPos = new Vector3(0, -1.5 * tileSide, 90);

    public void initTeleop() {
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
        PositionTrackerSettings pts = new PositionTrackerSettings(AxesOrder.XYZ, false, 100, new Vector3(2, 2, 2), fieldStartPos);
        pt = new PositionTracker(robot, pts, PositionTrackerHardware.makeDefault(robot));
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

        while (opModeIsActive()) {
            start = System.currentTimeMillis();
            robot.run();
            telemetry.addData("position", pt.getCurrentPosition());
            telemetry.addData("tile position", fieldToTile(pt.getCurrentPosition()));
            telemetry.addData("relative position", pt.getRelativePosition());
            telemetry.addData("spinner RPM", launcher.getRPM());
            telemetry.addData("encoder value", launcher.getHardware().launcherMotor.getCurrentPosition());
            telemetry.addData("Disk Position", diskFinder.getDiskPos());
            telemetry.addData("Door Servo", launcher.getHardware().doorServo.getPosition());
            telemetry.addData("loop time", System.currentTimeMillis() - start);
//            dashboard.sendTelemetryPacket(packet);
            telemetry.update();
        }
        robot.stop();
    }

    public void buildATagLibrary() {
        // https://ftc-docs.firstinspires.org/en/latest/apriltag/vision_portal/apriltag_library/apriltag-library.html
        AprilTagLibrary.Builder myAprilTagLibraryBuilder;
        AprilTagProcessor.Builder myAprilTagProcessorBuilder;
        AprilTagLibrary myAprilTagLibrary;
        AprilTagProcessor myAprilTagProcessor;

        // Create a new AprilTagLibrary.Builder object and assigns it to a variable.
        myAprilTagLibraryBuilder = new AprilTagLibrary.Builder();

        // Add all the tags from the given AprilTagLibrary to the AprilTagLibrary.Builder.
        // Get the AprilTagLibrary for the current season.
        myAprilTagLibraryBuilder.addTags(AprilTagGameDatabase.getCurrentGameTagLibrary());

        // Add a tag, without pose information, to the AprilTagLibrary.Builder.
        myAprilTagLibraryBuilder.addTag(55, "Our Awesome Team Tag", 3.5, DistanceUnit.INCH);

        // Build the AprilTag library and assign it to a variable.
        myAprilTagLibrary = myAprilTagLibraryBuilder.build();

        // Create a new AprilTagProcessor.Builder object and assign it to a variable.
        myAprilTagProcessorBuilder = new AprilTagProcessor.Builder();

        // Set the tag library.
        myAprilTagProcessorBuilder.setTagLibrary(myAprilTagLibrary);

        // Build the AprilTag processor and assign it to a variable.
        myAprilTagProcessor = myAprilTagProcessorBuilder.build();
    }
}


