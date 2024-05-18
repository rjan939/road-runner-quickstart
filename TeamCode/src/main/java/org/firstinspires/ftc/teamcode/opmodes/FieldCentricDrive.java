package org.firstinspires.ftc.teamcode.opmodes;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.PoseVelocity2d;
import com.acmerobotics.roadrunner.Rotation2d;
import com.acmerobotics.roadrunner.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.helpers.ActionOpMode;
import org.firstinspires.ftc.teamcode.helpers.PoseStorage;

@TeleOp(name="TeleOp Field Centric")
public class FieldCentricDrive extends ActionOpMode {

    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        drive = new MecanumDrive(hardwareMap, PoseStorage.currentPose);

        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive() && !isStopRequested()) {
            Pose2d poseEstimate = drive.pose;
            Vector2d input = new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            );
            input = Rotation2d.fromDouble(-poseEstimate.heading.log() - Math.toRadians(90))
                    .times(new Vector2d(input.x, input.y));
            drive.setDrivePowers(
                    new PoseVelocity2d(
                            new Vector2d(
                                    input.x,
                                    input.y
                            ),
                            gamepad1.right_stick_x
                    )
            );
        }

    }
}
