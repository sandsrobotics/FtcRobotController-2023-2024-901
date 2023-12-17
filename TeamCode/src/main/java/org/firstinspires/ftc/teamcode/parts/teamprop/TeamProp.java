package org.firstinspires.ftc.teamcode.parts.teamprop;

import com.qualcomm.robotcore.hardware.HardwareMap;
import org.apache.commons.lang3.ObjectUtils;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvInternalCamera;
import om.self.ezftc.core.Robot;
import om.self.ezftc.core.part.LoopedPartImpl;

public class TeamProp extends LoopedPartImpl<Robot, ObjectUtils.Null, ObjectUtils.Null> {
    OpenCvCamera camera;
    public TeamPropDetectionPipeline pipeline;
 //   public TeamPropDetectionPipeline.TeamPropPosition position;

    public TeamProp(Robot parent) {
        super(parent, "tags");
    }

    @Override
    public void onBeanLoad() {}

    @Override
    public void onInit() {
        HardwareMap hardwareMap = parent.opMode.hardwareMap;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Webcam 1"), cameraMonitorViewId);
         pipeline = new TeamPropDetectionPipeline();
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
        pipeline.position = pipeline.getAnalysis();

    }

    @Override
    public void onRun() {
    }

    @Override
    public void onStop() {}
}
