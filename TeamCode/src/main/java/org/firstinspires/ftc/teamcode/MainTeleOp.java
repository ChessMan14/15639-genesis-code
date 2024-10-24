package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@TeleOp(name="Main TeleOp", group="Linear OpMode")
public class MainTeleOp extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left_motor = null;
    private DcMotor back_left_motor = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_right_motor = null;

    @Override
    public void runOpMode() {
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");

        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        runtime.reset();

        while (opModeIsActive()) {
            double max;

            double axial = -gamepad1.left_stick_y;
            double lateral = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            double front_left_power = axial + lateral + yaw;
            double front_right_power = axial - lateral - yaw;
            double back_left_power = axial - lateral + yaw;
            double back_right_power = axial + lateral - yaw;

            max = Math.max(Math.abs(front_left_power), Math.abs(front_right_power));
            max = Math.max(max, Math.abs(back_left_power));
            max = Math.max(max, Math.abs(back_right_power));

            if (max > 1.0) {
                front_left_power  /= max;
                front_right_power /= max;
                back_left_power   /= max;
                back_right_power  /= max;
            }

            front_left_motor.setPower(front_left_power);
            front_right_motor.setPower(front_right_power);
            back_left_motor.setPower(back_left_power);
            back_right_motor.setPower(back_right_power);

            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", front_left_power, front_right_power);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", back_left_power, back_right_power);
            telemetry.update();
        }
    }
}