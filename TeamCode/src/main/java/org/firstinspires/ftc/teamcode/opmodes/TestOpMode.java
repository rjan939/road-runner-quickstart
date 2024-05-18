package org.firstinspires.ftc.teamcode.opmodes;

import android.util.Size;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.ProfileAccelConstraint;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.AprilTagDrive;
import org.firstinspires.ftc.teamcode.helpers.ActionHelpers.RaceParallelCommand;
import org.firstinspires.ftc.teamcode.helpers.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;
import org.firstinspires.ftc.teamcode.helpers.VisionHelper;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

@Autonomous(name="AprilTagShowcase")
@Config

public class TestOpMode extends ActionOpMode {
    private AprilTagDrive drive;


    @Override
    public void runOpMode() throws InterruptedException {

        VisionHelper vision = new VisionHelper(hardwareMap);

        FtcDashboard.getInstance().startCameraStream(vision.cameraStreamProcessor, 30);

        AprilTagDrive drive = new AprilTagDrive(hardwareMap,
                new Pose2d(-60, -36, Math.toRadians(180)), vision.aprilTag);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            Action traj =
                    drive.actionBuilder(drive.pose)
                            .splineTo(new Vector2d(45,-36), Math.toRadians(0),
                                    null,
                                    new ProfileAccelConstraint(-20.0, 20))
                            .turn(180)
                            .setTangent(Math.toRadians(180))
                            .splineTo(new Vector2d(-60,-36), Math.toRadians(180),
                                    null,
                                    new ProfileAccelConstraint(-20.0, 20))
                            .turn(180)
                            .build();


            Actions.runBlocking(new RaceParallelCommand(
                    traj
                    // You can update pid here or do anything at the same time this traj runs
            ));
            telemetry.addData("x: ", drive.pose.position.x);
            telemetry.addData("y: ", drive.pose.position.y);
            telemetry.addData("heading: ", drive.pose.heading);
        }
        PoseStorage.currentPose = drive.pose;
    }
}
