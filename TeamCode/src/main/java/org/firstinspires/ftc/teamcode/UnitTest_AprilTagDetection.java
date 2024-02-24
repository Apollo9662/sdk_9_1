package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessorImpl;

@TeleOp(name="UnitTest April detection", group="Unit Test")
public class UnitTest_AprilTagDetection extends LinearOpMode
{
    enum TagProcessorState {SET_DRAW_CUBE_PROJECTION,
                            SET_DRAW_TAG_ID,
                            SET_DRAW_AXES,
                            SET_DRAW_TAG_OUT_LINE}
    boolean press = false;
    TagProcessorState tagProcessorState;
    @Override
    public void runOpMode() throws InterruptedException
    {
        tagProcessorState = TagProcessorState.SET_DRAW_CUBE_PROJECTION;
        AprilTagProcessor tagProcessor = new AprilTagProcessor.Builder()
                .setDrawCubeProjection(true)
                .build();
        VisionPortal visionPortal = new VisionPortal.Builder()
                .addProcessor(tagProcessor)
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .setCameraResolution(new Size(640,  480))
                .build();

        waitForStart();
        while ((!isStopRequested()) && (opModeIsActive()))
        {
            if (tagProcessor.getDetections().size() > 0)
            {
                AprilTagDetection tag = tagProcessor.getDetections().get(0);
                telemetry.addData("x" ,tag.ftcPose.x);
                telemetry.addData("y" , tag.ftcPose.y);
                telemetry.addData("z", tag.ftcPose.z);
                telemetry.addData("pitch", tag.ftcPose.pitch);
                telemetry.addData("roll" ,tag.ftcPose.roll);
                telemetry.addData("yaw", tag.ftcPose.yaw);
                telemetry.addData("range" ,tag.ftcPose.range);
                telemetry.addData("bearing", tag.ftcPose.bearing);
                telemetry.addData("elevation" ,tag.ftcPose.elevation);
            }
            telemetry.update();
            if (gamepad1.b == true)
            {
                if (press == false)
                {
                    press = true;
                    switch (tagProcessorState)
                    {
                        case  SET_DRAW_CUBE_PROJECTION:
                            tagProcessorState = TagProcessorState.SET_DRAW_TAG_ID;
                            tagProcessor = new AprilTagProcessor.Builder()
                                    .setDrawCubeProjection(true)
                                    .build();
                            break;
                        case SET_DRAW_TAG_ID:
                            tagProcessorState = TagProcessorState.SET_DRAW_AXES;
                            tagProcessor = new AprilTagProcessor.Builder()
                                    .setDrawTagID(true)
                                    .build();
                            break;
                        case SET_DRAW_AXES:
                            tagProcessorState = TagProcessorState.SET_DRAW_TAG_OUT_LINE;
                            tagProcessor = new AprilTagProcessor.Builder()
                                    .setDrawAxes(true)
                                    .build();
                            break;
                        case  SET_DRAW_TAG_OUT_LINE:
                            tagProcessorState = TagProcessorState.SET_DRAW_CUBE_PROJECTION;
                            tagProcessor = new AprilTagProcessor.Builder()
                                    .setDrawTagOutline(true)
                                    .build();
                    }
                }

            }
            else if (gamepad1.x== true)
            {
                if (press == false)
                {
                    press = true;
                    switch (tagProcessorState)
                    {
                        case  SET_DRAW_CUBE_PROJECTION:
                            tagProcessorState = TagProcessorState.SET_DRAW_TAG_OUT_LINE;
                            tagProcessor = new AprilTagProcessor.Builder()
                                    .setDrawCubeProjection(true)
                                    .build();
                            break;
                        case SET_DRAW_TAG_ID:
                            tagProcessorState = TagProcessorState.SET_DRAW_CUBE_PROJECTION;
                            tagProcessor = new AprilTagProcessor.Builder()
                                    .setDrawTagID(true)
                                    .build();
                            break;
                        case SET_DRAW_AXES:
                            tagProcessorState = TagProcessorState.SET_DRAW_TAG_ID;
                            tagProcessor = new AprilTagProcessor.Builder()
                                    .setDrawAxes(true)
                                    .build();
                            break;
                        case  SET_DRAW_TAG_OUT_LINE:
                            tagProcessorState = TagProcessorState.SET_DRAW_AXES;
                            tagProcessor = new AprilTagProcessor.Builder()
                                    .setDrawTagOutline(true)
                                    .build();
                    }
                }

            }
            else
            {
                press = false;
            }
        }
    }
}
