package org.firstinspires.ftc.teamcode.parts.apriltag;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.depricated.lifter.Lifter;
import org.firstinspires.ftc.teamcode.parts.drive.Drive;
import org.firstinspires.ftc.teamcode.parts.drive.DriveControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;

import java.util.List;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

public class AprilTag extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
    OpenCvCamera camera;
    private Drive drive;
    public AprilTagDetectionPipeline pipeline;
    private static final boolean USE_WEBCAM = true;  // Set true to use a webcam, or false for a phone camera
    private int DESIRED_TAG_ID = -1;     // Choose the tag you want to approach or set to -1 for ANY tag.
    private VisionPortal visionPortal;               // Used to manage the video source.
    private AprilTagProcessor aprilTag;              // Used for managing the AprilTag detection process.
    public AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag
    public boolean targetFound = false;
    public List<AprilTagDetection> currentDetections;

    /**
     * A constructor for Part that creates task and event managers attached to the parents task and event managers. This also create a loop that will run forever
     *
     * @param parent the parent of this part(CAN NOT BE NULL)
     */

    public AprilTag(Robot parent) {
        super(parent, "tag");
    }

    @Override
    public void onRun() {
        if(aprilTag.getDetections() != null){
            currentDetections = aprilTag.getDetections();
            for (AprilTagDetection detection : currentDetections) {
                // Look to see if we have size info on this tag.
                if (detection.metadata != null) {
                    //  Check to see if we want to track towards this tag.
                    if ((DESIRED_TAG_ID < 0) || (detection.id == DESIRED_TAG_ID)) {
                        // Yes, we want to use this tag.
                        targetFound = true;
                        desiredTag = detection;
                        break;  // don't look any further.
                    } else {
                        // This tag is in the library, but we do not want to track it right now.
                    }
                } else {
                    // This tag is NOT in the library, so we don't have enough information to track to it.
                }
            }
        }
        else{
            targetFound = false;
            desiredTag = null;
        }
    }

    @Override
    public void onBeanLoad() {}

    /**
     * WARNING: beans may not be loaded onInit, please use onStart for beans
     */
    @Override
    public void onInit() {
        // Create the AprilTag processor the easy way.
        aprilTag = AprilTagProcessor.easyCreateWithDefaults();
        visionPortal = VisionPortal.easyCreateWithDefaults(parent.opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), aprilTag);
        /*
        initAprilTag();
        int cameraMonitorViewId = parent.opMode.hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", parent.opMode.hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(parent.opMode.hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
        pipeline = new AprilTagDetectionPipeline();
        camera.setPipeline(pipeline);
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                camera.startStreaming(1280,720, OpenCvCameraRotation.SENSOR_NATIVE);
            }

            @Override
            public void onError(int errorCode)
            {
                 // This will be called if the camera could not be opened
            }
        });
        */
    }

    @Override
    public void onStart() {
    }

    @Override
    public void onStop() {
        //visionPortal.close();
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Adjust Image Decimation to trade-off detection-range for detection-rate.
        // eg: Some typical detection data using a Logitech C920 WebCam
        // Decimation = 1 ..  Detect 2" Tag from 10 feet away at 10 Frames per second
        // Decimation = 2 ..  Detect 2" Tag from 6  feet away at 22 Frames per second
        // Decimation = 3 ..  Detect 2" Tag from 4  feet away at 30 Frames Per Second
        // Decimation = 3 ..  Detect 5" Tag from 10 feet away at 30 Frames Per Second
        // Note: Decimation can be changed on-the-fly to adapt during a match.
        aprilTag.setDecimation(2);

        // Create the vision portal by using a builder.
        if (USE_WEBCAM) {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(parent.opMode.hardwareMap.get(WebcamName.class, "Webcam 1"))
                    .addProcessor(aprilTag)
                    .build();
        } else {
            visionPortal = new VisionPortal.Builder()
                    .setCamera(BuiltinCameraDirection.BACK)
                    .addProcessor(aprilTag)
                    .build();
        }
    }

    public void setDesiredTag(int tag){
        DESIRED_TAG_ID = tag;
    }
}
