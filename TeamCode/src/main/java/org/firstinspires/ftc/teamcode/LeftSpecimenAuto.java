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

import java.util.HashMap;

@Autonomous(name="Left Specimen Auto", group="Robot")
public class LeftSpecimenAuto extends LinearOpMode {
    //Create the variables for the motors and servos and init
    // ializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();

    //Map of all motors
    private HashMap<String, DcMotor> motors = new HashMap<>();
    //Map of wheel motor powers
    private HashMap<String, Double> wheel_motor_powers = new HashMap<>();
    //Map of arm motor powers
    private HashMap<String, Double> other_motor_powers = new HashMap<>();

    //Map of all servos
    private HashMap<String, Servo> servos = new HashMap<>();
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
        double slide_servo_setting = 0.75;
        double arm_servo_setting = 0;

        //Set slide servo
        servos.get("slide_servo").setPosition(0.75);

        //Initialize AutoMover
        AutoMover autoMover = new AutoMover(motors.get("front_left"), motors.get("back_left"), motors.get("front_right"), motors.get("back_right"));

        //Movement is in cm, rotation is in degrees

        //Wait for the other robot to complete their autonomous and get out of the way first
        sleep(15000);

        //Robot needs to move forward a bit so that it has space to rotate (cm)
        //(left/counterclockwise is negative; (distance, degree))
        autoMover.move(30, 0);
        autoMover.move(145, 90);

        //Raise slide
        motors.get("slide").setPower(1);
        sleep(1500);
        motors.get("slide").setPower(0);

        autoMover.move(90, 0);

        //Lower slide
        motors.get("slide").setPower(0);
        sleep(500);
        autoMover.move(10, 0);
        motors.get("slide").setPower(-0.5);
        sleep(500);
        motors.get("slide").setPower(0);

        double current_time = runtime.seconds();

        sleep(1000);

        //Move back again
        motors.get("slide").setPower(-0.5);
        autoMover.move(50, 180);
    }
}