/*
MIT License

Copyright (c) 2024 ChessMan14

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//This decorator puts this opmode into selected the name and group on the driver hub menu
@TeleOp(name="Main TeleOp", group="Linear OpMode")
//Since java is weird, this is essentially the equivalent of a main method in C, but instead it's a class. Also, we "extend" this class from the library class LinearOpMode which makes this into a proper teleop opmode
public class MainTeleOp extends LinearOpMode {

    //Create the variables for the motors and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left_motor = null;
    private DcMotor back_left_motor = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_right_motor = null;

    //Global speed percentage for all movement
    private final double speed_coefficient = 1;

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    public void runOpMode() {
        //Map the actual physical motors to the variables. The "device_name" variable is set in the driver hub configuration
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");

        //Set direction of motors
        front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        back_right_motor.setDirection(DcMotor.Direction.FORWARD);

        //This data is displayed on the driver hub console
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        //Reset runtime var
        runtime.reset();

        //Main loop. This runs until stop is pressed on the driver hub
        while (opModeIsActive()) {
            //Hold the maximum power being applied to a single wheel
            double max;

            //Constant variable for rotation speed
            final double rotate_fact = 1;

            //Vertical movement (up is negative on the joystick)
            double axial = -gamepad1.left_stick_y;
            //Horizontal movement
            double lateral = gamepad1.left_stick_x;
            //Rotation calculation
            double yaw = rotate_fact*(-gamepad1.left_trigger + gamepad1.right_trigger);

            //Calculate how much power to send to each wheel based on vertical/horizontal movement and rotation
            double front_left_power = axial + lateral + yaw;
            double front_right_power = axial - lateral - yaw;
            double back_left_power = axial - lateral + yaw;
            double back_right_power = axial + lateral - yaw;

            //Find the maximum power being applied to a single wheel
            max = Math.max(Math.abs(front_left_power), Math.abs(front_right_power));
            max = Math.max(max, Math.abs(back_left_power));
            max = Math.max(max, Math.abs(back_right_power));

            //If power > 100%, scale down all the power variables.
            if (max > 1.0) {
                front_left_power  /= max;
                front_right_power /= max;
                back_left_power   /= max;
                back_right_power  /= max;
            }

            //Send power to the motors
            front_left_motor.setPower(front_left_power*speed_coefficient);
            front_right_motor.setPower(front_right_power*speed_coefficient);
            back_left_motor.setPower(back_left_power*speed_coefficient);
            back_right_motor.setPower(back_right_power*speed_coefficient);

            //Display data on driver hub
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", front_left_power, front_right_power);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", back_left_power, back_right_power);
            telemetry.update();
        }
    }
}