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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

@Autonomous(name="Left Basket Autonomous", group="Robot")
public class LeftBasketAutonomous extends LinearOpMode {
    //Create the variables for the motors and servos and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();

    //Map of all motors
    private HashMap<String, DcMotor> motors = new HashMap<>();
    //Map of wheel motor powers
    private HashMap<String, Double> wheel_motor_powers = new HashMap<>();
    //Map of arm motor powers
    private HashMap<String, Double> other_motor_powers = new HashMap<>();

    //Map of all servos
    private HashMap<String, Servo> servos = new HashMap<>();
    //Map of all servos
    private HashMap<String, CRServo> crservos = new HashMap<>();
    //Map of all servos positions
    private HashMap<String, Double> servo_positions = new HashMap<>();

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    //Throws condition is there for the later wait() command; shouldn't ever be an error though
    public void runOpMode() {
        //Create and assign map entries for all motors
        motors.put("front_left", hardwareMap.get(DcMotor.class, "front_left_motor"));
        motors.put("back_left", hardwareMap.get(DcMotor.class, "back_left_motor"));
        motors.put("front_right", hardwareMap.get(DcMotor.class, "front_right_motor"));
        motors.put("back_right", hardwareMap.get(DcMotor.class, "back_right_motor"));

        motors.put("slide", hardwareMap.get(DcMotor.class, "slide_motor"));
        motors.put("arm", hardwareMap.get(DcMotor.class, "arm_motor"));
        //motors.put("actuator", hardwareMap.get(DcMotor.class, "actuator_motor"));

        //Create and assign map entries for all servos
        servos.put("slide_servo", hardwareMap.get(Servo.class, "slide_servo"));
        servos.put("arm_servo", hardwareMap.get(Servo.class, "arm_servo"));

        crservos.put("claw_servo", hardwareMap.get(CRServo.class, "rotator_servo"));

        //Set direction of motors
        motors.get("front_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("back_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("front_right").setDirection(DcMotor.Direction.FORWARD);
        motors.get("back_right").setDirection(DcMotor.Direction.FORWARD);

        motors.get("slide").setDirection(DcMotor.Direction.REVERSE);
        motors.get("arm").setDirection(DcMotor.Direction.FORWARD);
        //motors.get("actuator").setDirection(DcMotor.Direction.FORWARD);

        //Set direction of servos
        servos.get("slide_servo").setDirection(Servo.Direction.FORWARD);
        servos.get("arm_servo").setDirection(Servo.Direction.REVERSE);

        //Turn on the brakes for 0 power
        for (String key : motors.keySet()) {
            motors.get(key).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
        }

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

        //Grip sample
        arm_servo_setting = 0.15;
        servos.get("arm_servo").setPosition(arm_servo_setting);

        //close basket servo
        slide_servo_setting = 0.75;
        servos.get("slide_servo").setPosition(slide_servo_setting);

        //Initialize AutoMover
        AutoMover autoMover = new AutoMover(motors.get("front_left"), motors.get("back_left"), motors.get("front_right"), motors.get("back_right"));

        while (opModeIsActive()) {
            //Movement is in cm, rotation is in degrees

            //Robot needs to move forward a bit so that it has space to rotate (cm)
            //(left/counterclockwise is negative; (distance, degree))
            autoMover.move(30, 0);
            autoMover.rotate(-150);
            autoMover.move(65, 90);

            double current_time = runtime.seconds();

            //make slide go up
            motors.get("slide").setPower(1);

            sleep(2000);

            //Move forward a bit
            autoMover.move(100, 0);

            sleep(300);

            //Release sample
            motors.get("slide").setPower(0.1);
            slide_servo_setting = 0.1;
            servos.get("slide_servo").setPosition(slide_servo_setting);
            sleep(1000);
            slide_servo_setting = 1;
            servos.get("slide_servo").setPosition(slide_servo_setting);
            sleep(400);
            slide_servo_setting = 0.1;
            servos.get("slide_servo").setPosition(slide_servo_setting);
            sleep(1000);

            autoMover.move(90, 180);

            //Close basket
            slide_servo_setting = 1;
            servos.get("slide_servo").setPosition(slide_servo_setting);

            //make slide go down
            motors.get("slide").setPower(-1);
            sleep(400);
            motors.get("slide").setPower(1);
            sleep(300);
            motors.get("slide").setPower(-1);
            sleep(1000);
            motors.get("slide").setPower(0);

            autoMover.rotate(145);
            autoMover.move(25, -90);
            autoMover.move(10, 0);


            motors.get("arm").setPower(0.5);

            sleep(250);
            servos.get("arm_servo").setPosition(0.75);

            sleep(500);

            motors.get("arm").setPower(-0.1);

            servos.get("arm_servo").setPosition(0.25);
            motors.get("arm").setPower(0);
            sleep(500);

            motors.get("arm").setPower(-0.5);
            sleep(3000);

            motors.get("arm").setPower(0.12);
            crservos.get("claw_servo").setPower(-0.75);
            sleep(350);

            servos.get("arm_servo").setPosition(0.60);
            crservos.get("claw_servo").setPower(0);

            motors.get("arm").setPower(0);
            sleep(350);
            autoMover.rotate(-170);
            motors.get("slide").setPower(1);

            autoMover.move(100, -15);

            sleep(1500);
            motors.get("slide").setPower(0.1);
            //Release sample
            slide_servo_setting = 0.15;
            servos.get("slide_servo").setPosition(slide_servo_setting);

            sleep(1000);

            //Release sample
            motors.get("slide").setPower(0.1);
            slide_servo_setting = 0.1;
            servos.get("slide_servo").setPosition(slide_servo_setting);
            sleep(1000);
            slide_servo_setting = 1;
            servos.get("slide_servo").setPosition(slide_servo_setting);
            sleep(400);
            slide_servo_setting = 0.1;
            servos.get("slide_servo").setPosition(slide_servo_setting);
            sleep(1000);

            break;
        }
    }
}