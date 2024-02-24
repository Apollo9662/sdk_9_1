package org.firstinspires.ftc.teamcode;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;


@TeleOp(name="controler Test", group="Unit Test")
//@Disabled
public class controlerTest extends LinearOpMode
{
    public void runOpMode()
    {
        boolean dpad_up = false;
        boolean dpad_right = false;
        boolean dpad_left = false;
        boolean dpad_down = false;
        boolean A = gamepad1.a;
        boolean B = gamepad1.b;
        boolean X = gamepad1.x;
        boolean Y = gamepad1.y;
        float left_trigger = gamepad1.left_trigger;
        float right_trigger = gamepad1.right_trigger;
        boolean right_bumper = gamepad1.right_bumper;
        boolean left_bumper = gamepad1.left_bumper;
        float right_stick_x = gamepad1.right_stick_x;
        float right_stick_y = gamepad1.right_stick_y;
        float left_stick_x = gamepad1.left_stick_x;
        float left_stick_y = gamepad1.left_stick_y;

        waitForStart();

        while (opModeIsActive())
        {
             dpad_up = gamepad1.dpad_up;
             dpad_down = gamepad1.dpad_down;
             dpad_left = gamepad1.dpad_left;
             dpad_right = gamepad1.dpad_right;
             A = gamepad1.a;
             B = gamepad1.b;
             X = gamepad1.x;
             Y = gamepad1.y;
            left_trigger = gamepad1.left_trigger;
            right_trigger = gamepad1.right_trigger;
            right_bumper = gamepad1.right_bumper;
            left_bumper = gamepad1.left_bumper;
            right_stick_x = gamepad1.right_stick_x;
            right_stick_y = gamepad1.right_stick_y;
            left_stick_x = gamepad1.left_stick_x;
            left_stick_y = gamepad1.left_stick_y;

            if (dpad_up == true)
            {
                telemetry.addLine("dpad_up press");
            }


            if (dpad_down == true)
            {
                telemetry.addLine("dpad_down press");
            }


            if (dpad_left == true)
            {
                telemetry.addLine("dpad_left press");
            }


            if (dpad_right == true)
            {
                telemetry.addLine("dpad_right press");
            }

            if (A == true)
            {
                telemetry.addLine("A press");
            }

            if (B == true)
            {
                telemetry.addLine("B press");
            }

            if (X == true)
            {
                telemetry.addLine("X press");
            }

            if (Y == true)
            {
                telemetry.addLine("Y press");
            }

            if (left_trigger != 0)
            {
                telemetry.addData("left trigger press", "%2.4f", left_trigger);
            }
            if (right_trigger != 0)
            {
                telemetry.addData("right trigger press", "%2.4f", right_trigger);
            }

            if (right_bumper == true)
            {
                telemetry.addLine("right bumper press");
            }
            if (left_bumper == true)
            {
                telemetry.addLine("left bumper press");
            }

            if (right_stick_x != 0)
            {
                telemetry.addData("right stick x press ", "%2.4f", right_stick_x);
            }

            if (right_stick_y != 0)
            {
                telemetry.addData("right stick y press ", "%2.4f", right_stick_y);
            }

            if (left_stick_x != 0)
            {
                telemetry.addData("right stick x press ", "%2.4f", left_stick_x);
            }

            if (left_stick_y != 0)
            {
                telemetry.addData("right stick x press ", "%2.4f", left_stick_y);
            }



        telemetry.update();
        }
    }

}
