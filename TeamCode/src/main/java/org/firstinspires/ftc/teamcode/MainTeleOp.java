/*
MIT License

Copyright (c) 2024 ChessMan14, angeldescended

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
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//This decorator puts this opmode into selected the name and group on the driver hub menu
@TeleOp(name="Main TeleOp", group="Linear OpMode")
//Since java is weird, this is essentially the equivalent of a main method in C, but instead it's a class. Also, we "extend" this class from the library class LinearOpMode which makes this into a proper teleop opmode
public class MainTeleOp extends LinearOpMode {

    //Create the variables for the motors and servos and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();

    private DcMotor front_left_motor = null;
    private DcMotor back_left_motor = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_right_motor = null;

    private DcMotor slide_motor = null;
    private DcMotor arm_motor = null;

    private Servo slide_servo = null;
    private Servo arm_servo = null;

    //Global speed percentage for all wheel movement
    private final double wheel_speed_coefficient = 0.5;

    //Speed percentage for slide
    private final double slide_speed_coefficient = 0.1;

    //Speed percentage for arm
    private final double arm_speed_coefficient = 0.1;

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    public void runOpMode() {
        //Wheel motors

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

        //Arm motors and servos

        //Map the actual physical motors to the variables. The "device_name" variable is set in the driver hub configuration
        slide_motor = hardwareMap.get(DcMotor.class, "slide_motor");
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");

        slide_servo = hardwareMap.get(Servo.class, "slide_servo");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");

        //TODO Fix all these directions

        //Set direction of motors
        slide_motor.setDirection(DcMotor.Direction.FORWARD);
        arm_motor.setDirection(DcMotor.Direction.FORWARD);

        //Set direction of servos
        arm_servo.setDirection(Servo.Direction.FORWARD);
        arm_servo.setDirection(Servo.Direction.FORWARD);

        //This data is displayed on the driver hub console
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        //Reset runtime var
        runtime.reset();

        //Main loop. This runs until stop is pressed on the driver hub
        while (opModeIsActive()) {
            //Movement

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
                front_left_power /= max;
                front_right_power /= max;
                back_left_power /= max;
                back_right_power /= max;
            }

            //Send power to the motors
            front_left_motor.setPower(front_left_power*wheel_speed_coefficient);
            front_right_motor.setPower(front_right_power*wheel_speed_coefficient);
            back_left_motor.setPower(back_left_power*wheel_speed_coefficient);
            back_right_motor.setPower(back_right_power*wheel_speed_coefficient);

            //Arm control

            //Java is stupid so this is ugly. slide_power will get 1 if y is pressed, -1 if a is pressed, and 0 if both or neither are pressed
            int slide_power = (gamepad2.y ? 1 : 0) - (gamepad2.a ? 1 : 0);

            //Triggers used so that the driver can move the arm slower if they want
            double arm_power = gamepad2.left_trigger - gamepad2.right_trigger;

            //Send power to motors
            slide_motor.setPower(slide_power);
            arm_motor.setPower(arm_power);

            //TODO Fix these servo rotations, i.e make them rotate by the proper amount

            //You can set a servo to a position from 0-1. This corresponds the servo turning to 0-180 degrees from adjacent to where the wires come out
            //If left bumper is pressed, set servo to 180 degrees. Otherwise, set it to 0
            double slide_servo_setting = gamepad2.left_bumper ? 1 : 0;

            //If b, set servo to 180 degrees. Otherwise, set it to 0
            double arm_servo_setting = gamepad2.b ? 1: 0;

            //Set servo positions
            slide_servo.setPosition(slide_servo_setting);
            arm_servo.setPosition(arm_servo_setting);

            //Display data on driver hub
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", front_left_power, front_right_power);
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", back_left_power, back_right_power);
            telemetry.update();
        }
    }
}