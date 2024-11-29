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

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@Autonomous(name="Autonomous 2", group="Robot")
public class Autonomous2 extends LinearOpMode {

    //Create the variables for the motors and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left_motor = null;
    private DcMotor back_left_motor = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_right_motor = null;

    private DcMotor slide_motor = null;
    private DcMotor arm_motor = null;

    private Servo slide_servo = null;
    private Servo arm_servo = null;


    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    //Throws condition is there for the later wait() command; shouldn't ever be an error though
    public void runOpMode() throws InterruptedException {
        //Map the actual physical motors to the variables. The "device_name" variable is set in the driver hub configuration
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");

        //Arm motors and servos

        //Map the actual physical motors to the variables. The "device_name" variable is set in the driver hub configuration
        slide_motor = hardwareMap.get(DcMotor.class, "slide_motor");
        arm_motor = hardwareMap.get(DcMotor.class, "arm_motor");

        slide_servo = hardwareMap.get(Servo.class, "slide_servo");
        arm_servo = hardwareMap.get(Servo.class, "arm_servo");

        //Set direction of motors
        slide_motor.setDirection(DcMotor.Direction.REVERSE);
        arm_motor.setDirection(DcMotor.Direction.FORWARD);

        //Set direction of servos
        slide_servo.setDirection(Servo.Direction.FORWARD);
        arm_servo.setDirection(Servo.Direction.FORWARD);

        //This data is displayed on the driver hub console
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        //Reset runtime var
        runtime.reset();

        //settings for servos
        double slide_servo_setting;
        double arm_servo_setting = 0;
        arm_servo.setPosition(arm_servo_setting);

        //Initialize AutoMover
        AutoMover autoMover = new AutoMover(front_left_motor, back_left_motor, front_right_motor, back_right_motor);

        //Movement is in cm, rotation is in degrees

        //Wait for the other robot to complete their autonomous and get out of the way first
        sleep(10000);

        //(left/counterclockwise is negative; (distance, degree))
        autoMover.move(70, 0);
        autoMover.move(240, -90);
        autoMover.move(50, -90);

        //TODO Implement arm movement to place sample
        //Tentatively marked complete; below code should do that

        double current_time = runtime.seconds();

        //make the arm go forward
        arm_motor.setPower(-1);

        while (runtime.seconds() < current_time + 0.75) {
            //do nothing
        }

        //stop making arm go forward
        arm_motor.setPower(0);

        //opens claw
        arm_servo_setting = (0.944);
        arm_servo.setPosition(arm_servo_setting);

    }
}
