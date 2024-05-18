package org.firstinspires.ftc.teamcode.helpers;

import android.util.Size;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.concurrent.TimeUnit;

public class VisionHelper {
    public AprilTagProcessor aprilTag;
    private WebcamName camera;
    public VisionPortal portal;
    public CameraStreamProcessor cameraStreamProcessor;

    public VisionHelper(HardwareMap hardwareMap) {
        aprilTag = new AprilTagProcessor.Builder()
                .setDrawAxes(true)
                .setDrawCubeProjection(true)
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                //Calibrated with 3df Zephyr, here are the default:
                //.setLensIntrinsics(822.317f, 822.317f, 319.495f, 242.502f)
                .setLensIntrinsics(566.710387614, 566.710387614, 402.459761739, 126.65056764)
                .build();
        aprilTag.setDecimation(2);
        cameraStreamProcessor = new CameraStreamProcessor();

        camera = hardwareMap.get(WebcamName.class, "Webcam 1");


        portal = new VisionPortal.Builder()
                .setStreamFormat(VisionPortal.StreamFormat.MJPEG)
                .setCamera(camera)
                .setCameraResolution(new Size(640, 480))
                .addProcessor(aprilTag)
                .enableLiveView(true)
                .build();
    }

    public boolean initLoop(Telemetry telemetry) {
        if (portal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addLine("Vision loading...");
            return true;
        }
        return false;
    }


}
