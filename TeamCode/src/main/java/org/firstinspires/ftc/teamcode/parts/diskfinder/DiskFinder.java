package org.firstinspires.ftc.teamcode.parts.diskfinder;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.ftccommon.SoundPlayer;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.parts.led.Led;
import org.firstinspires.ftc.teamcode.parts.positiontracker.PositionTracker;
import org.firstinspires.ftc.teamcode.parts.teamprop.TeamPropDetectionPipeline;
import org.firstinspires.ftc.vision.VisionPortal;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraException;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;
import om.self.ezftc.utils.Vector3;

public class DiskFinder extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
        OpenCvCamera camera;
        private VisionPortal visionPortal;
        public DiskFinderDetectionPipeline pipeline;
        public Boolean diskFound;
        public static Vector3 diskPos = null;
        private PositionTracker pt;
        private int goldSoundID;

        public DiskFinder(Robot parent) {
            super(parent, "disk finder");
        }

        @Override
        public void onBeanLoad() {}

        @Override
        public void onInit() {
//        visionPortal

            HardwareMap hardwareMap = parent.opMode.hardwareMap;
            goldSoundID = hardwareMap.appContext.getResources().getIdentifier("gold",   "raw", hardwareMap.appContext.getPackageName());
            int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
            camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
            FtcDashboard.getInstance().startCameraStream(camera, 10);
            pipeline = new DiskFinderDetectionPipeline();
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
                    /*
                     * This will be called if the camera could not be opened
                     */
                }
            });
        }

        @Override
        public void onStart() {
            pipeline.diskIsFound = pipeline.getAnalysis();
            pt = getBeanManager().getBestMatch(PositionTracker.class, false);
        }

        @Override
        public void onRun() {
            diskFound = pipeline.getAnalysis();
            if (diskFound & diskPos == null) {
                diskPos = pt.getCurrentPosition();
                SoundPlayer.getInstance().startPlaying(parent.opMode.hardwareMap.appContext, goldSoundID);
            }
        }

        @Override
        public void onStop() {
            if (camera.getClass() != null) {
            }
            try {
                if (camera != null) {
                    camera.stopStreaming();
                    camera.closeCameraDevice();
                }
            } catch (OpenCvCameraException E) {}
        }

        public static void clearDiskPos() {
            diskPos = null;
        }

        public static Vector3 getDiskPos() {
            return diskPos;
        }
    }

