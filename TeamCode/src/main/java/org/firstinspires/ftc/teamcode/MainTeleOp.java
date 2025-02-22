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
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;

//TODO FIX THE ROBOT FUCKERY
//This decorator puts this opmode into selected the name and group on the driver hub menu
@TeleOp(name="Main TeleOp", group="Linear OpMode")
//Since java is weird, this is essentially the equivalent of a main method in C, but instead it's a class. Also, we "extend" this class from the library class LinearOpMode which makes this into a proper teleop opmode
public class MainTeleOp extends LinearOpMode {
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
    //Map of all servos positions
    private HashMap<String, Double> servo_positions = new HashMap<>();

    //Map of all CRServos
    private HashMap<String, CRServo> crservos = new HashMap<>();
    //Map of all CRServo powers
    private HashMap<String, Double> crservo_powers = new HashMap<>();

    //Speed percentage for slide
    private final double slide_speed_coefficient = 0.70;

    //Speed percentage for arm
    private final double arm_speed_coefficient = 0.12;

    //Speed percentage for claw CRServo
    private final double rotator_servo_coefficient = 0.75;

    private final double actuator_speed_coefficient = 1.00;

    //Max wheel speed
    private final double max_wheel_speed = 0.5;

    //Constant variable for rotation speed
    final double rotate_fact = 0.75;

    //Counts per revolution (encoder thing)
    //See https://www.gobilda.com/5202-series-yellow-jacket-planetary-gear-motor-26-9-1-ratio-223-rpm-3-3-5v-encoder/
    final double CPR = 751.8;

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    public void runOpMode() {
        //Create and assign map entries for all motors
        motors.put("front_left", hardwareMap.get(DcMotor.class, "front_left_motor"));
        motors.put("back_left", hardwareMap.get(DcMotor.class, "back_left_motor"));
        motors.put("front_right", hardwareMap.get(DcMotor.class, "front_right_motor"));
        motors.put("back_right", hardwareMap.get(DcMotor.class, "back_right_motor"));

        motors.put("slide", hardwareMap.get(DcMotor.class, "slide_motor"));
        motors.put("arm", hardwareMap.get(DcMotor.class, "arm_motor"));
        motors.put("actuator", hardwareMap.get(DcMotor.class, "actuator_motor"));

        //Create and assign map entries for all servos
        servos.put("slide_servo", hardwareMap.get(Servo.class, "slide_servo"));
        servos.put("arm_servo", hardwareMap.get(Servo.class, "arm_servo"));

        //Create and assign map entries for all CRServos
        crservos.put("rotator_servo", hardwareMap.get(CRServo.class, "rotator_servo"));

        //Reset encoders
        for (String key : motors.keySet()) {
            motors.get(key).setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.get(key).setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        //Set direction of motors
        motors.get("front_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("back_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("front_right").setDirection(DcMotor.Direction.FORWARD);
        motors.get("back_right").setDirection(DcMotor.Direction.FORWARD);

        motors.get("slide").setDirection(DcMotor.Direction.REVERSE);
        motors.get("arm").setDirection(DcMotor.Direction.FORWARD);
        motors.get("actuator").setDirection(DcMotor.Direction.FORWARD);

        //Turn on the brakes for 0 power
        for (String key : motors.keySet()) {
            motors.get(key).setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);;
        }

        //Set direction of servos
        servos.get("slide_servo").setDirection(Servo.Direction.FORWARD);
        servos.get("arm_servo").setDirection(Servo.Direction.REVERSE);

        //This data is displayhed on the driver hub console
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        //Reset runtime var
        runtime.reset();

        //Hold the maximum power being applied to a single wheel
        double max;

        //initializes variables for the main loop

        //Vertical movement
        double axial;
        //Horizontal movement
        double lateral;
        //Rotation
        double yaw;

        //Motor powers
        wheel_motor_powers.put("front_left", 0.0);
        wheel_motor_powers.put("front_right", 0.0);
        wheel_motor_powers.put("back_left", 0.0);
        wheel_motor_powers.put("back_right", 0.0);

        other_motor_powers.put("slide", 0.0);
        other_motor_powers.put("arm", 0.0);
        other_motor_powers.put("actuator", 0.0);

        //Settings for servos
        servo_positions.put("slide_servo", 0.0);
        servo_positions.put("arm_servo", 0.0);

        //CRServos Powers
        crservo_powers.put("rotator_servo", 0.0);

        //initializes variable for later
        boolean arm_servo_initialized = false;

        //Speed percentage for all wheel movement
        double wheel_speed = 0;

        //The time (in seconds) when the robot started accelerating (-1 if not accelerating)
        double accel_start_time = -1;

        //The time when the robot will experience the next boost in speed from acceleration
        double accel_goal = 0;

        //Acceleration value for wheels (percent per second). Default is fast mode
        double wheel_accel = 0.015/0.1;

        //Macro to raise arm and drop sample in bucket
        boolean raise_drop_macro_running = false;
        double raise_drop_macro_goal = -1;

        boolean sensor_led_status = true;

        //What percent of normal power we apply to the arm when going down
        double arm_down_coefficient = 0.75;

        //Main loop. This runs until stop is pressed on the driver hub
        while (opModeIsActive()) {
            //Movement

            //Vertical movement (up is negative on the joystick)
            axial = -gamepad1.left_stick_y;
            //Horizontal movement
            lateral = gamepad1.left_stick_x;
            //Rotation calculation
            if ((gamepad1.right_trigger + gamepad1.left_trigger) > 0) {
                yaw = rotate_fact*(-gamepad1.left_trigger + gamepad1.right_trigger);
            }
            else {
                yaw = rotate_fact * (gamepad1.right_stick_x);
            }

            //Calculate how much power to send to each wheel based on vertical/horizontal movement and rotation
            wheel_motor_powers.put("front_left", axial + lateral + yaw);
            wheel_motor_powers.put("front_right", axial - lateral - yaw);
            wheel_motor_powers.put("back_left", axial - lateral + yaw);
            wheel_motor_powers.put("back_right", axial + lateral - yaw);

            //Find the maximum power being applied to a single wheel
            max = Math.max(Math.abs(wheel_motor_powers.get("front_left")), Math.abs(wheel_motor_powers.get("front_right")));
            max = Math.max(max, Math.abs(wheel_motor_powers.get("back_left")));
            max = Math.max(max, Math.abs(wheel_motor_powers.get("back_right")));

            //If power > 100%, scale down all the power variables.
            if (max > 1.0) {
                double final_max = max;
                wheel_motor_powers.replaceAll((key, val) -> val/final_max);
            }

            //Cancel arm macro
            if (gamepad2.dpad_right) {
                raise_drop_macro_running = false;
            }

            //Arm macro
            /*if (gamepad2.dpad_left || raise_drop_macro_running) {
                //Raise arm
                if (raise_drop_macro_goal < 0) {
                    raise_drop_macro_goal = runtime.seconds() + 1.0;
                    other_motor_powers.put("arm", -0.15);
                    raise_drop_macro_running = true;
                }
                //Cleanup
                else if (raise_drop_macro_goal + 2.5 < runtime.seconds()) {
                    other_motor_powers.put("arm", 0.0);
                    raise_drop_macro_running = false;
                    raise_drop_macro_goal = -1;
                }
                //Stop moving and drop sample
                else if (raise_drop_macro_goal + 1.5 < runtime.seconds()) {
                    other_motor_powers.put("arm", -0.1);
                    servo_positions.put("arm_servo", 0.60);
                    servos.get("arm_servo").setPosition(servo_positions.get("arm_servo"));
                }
                //Move arm slightly forward
                else if (raise_drop_macro_goal < runtime.seconds()) {
                    other_motor_powers.put("arm", 0.15);
                }
            }*/

            //Check for accel mode changes
            if (gamepad1.dpad_up) {
                wheel_accel = 0.010/0.1;
            }
            else if (gamepad1.dpad_down) {
                wheel_accel = 0.005/0.1;
            }

            //Calculate acceleration
            if (axial != 0 || lateral != 0 || yaw != 0) {
                //Robot was accelerating last cycle
                if (accel_start_time > 0) {
                    //Increase speed if past goal
                    if (runtime.seconds() >= accel_goal) {
                        if (wheel_speed < max_wheel_speed) {
                            wheel_speed += wheel_accel;
                        }
                        //Here we bank on the fact that the main loop runs at at least 10HZ, but according to Reddit it should run at 40HZ so we should be fine
                        accel_goal += 0.1;
                    }
                }
                //Robot was not accelerating last cycle
                else {
                    accel_start_time = runtime.seconds();
                    accel_goal = accel_start_time + 0.1;
                }
            }
            else {
                accel_start_time = -1;
                wheel_speed = 0;
            }

            //Send power to the motors
            double final_wheel_speed = wheel_speed;
            wheel_motor_powers.replaceAll((key, val) -> val*final_wheel_speed);

            //Arm control

            //Java is stupid so this is ugly. slide will get 1 if y is pressed, -1 if a is pressed, and 0 if both or neither are pressed
            other_motor_powers.put("slide", (double)((gamepad2.y ? 1 : 0) - (gamepad2.a ? 1 : 0))*slide_speed_coefficient);

            //Triggers used so that the driver can move the arm slower if they want
            if (!raise_drop_macro_running) {
                other_motor_powers.put("arm", (double) (((gamepad2.left_trigger*arm_down_coefficient) - gamepad2.right_trigger) * arm_speed_coefficient));
            }

            //D-pad up and down used for actuator movement
            if (gamepad2.dpad_up ^ gamepad2.dpad_down) {
                //Move actuator up
                if (gamepad2.dpad_up) {
                    other_motor_powers.put("actuator", actuator_speed_coefficient);
                }
                //Move it down
                else if (gamepad2.dpad_down) {
                    other_motor_powers.put("actuator", -actuator_speed_coefficient);
                }
            }
            else {
                other_motor_powers.put("actuator", 0.00);
            }

            //Send power to motors
            for (String key : motors.keySet()) {
                if (other_motor_powers.containsKey(key)) {
                    motors.get(key).setPower(other_motor_powers.get(key));
                }
                else {
                    motors.get(key).setPower(wheel_motor_powers.get(key));
                }
            }

            //Get input for rotator servo (left stick horizontal axis)
            crservo_powers.put("rotator_servo", gamepad2.left_stick_x*rotator_servo_coefficient);


            //You can set a servo to a position from 0-1. This corresponds the servo turning to 0-180 degrees from adjacent to where the wires come out
            //If left bumper is pressed, set servo to 180 degrees. Otherwise, set it to 0
            servo_positions.put("slide_servo", gamepad2.left_bumper ? 0.15 : 0.75);

            //Pressing b once opens servo, Pressing x once closes it. Do nothing if both are pressed
            if ((gamepad2.b ^ gamepad2.x) && !raise_drop_macro_running) {
                if (gamepad2.b) {
                    servo_positions.put("arm_servo", 0.60);
                    if (!arm_servo_initialized) {
                        arm_servo_initialized = true;
                    }
                }
                else if (gamepad2.x) {
                    servo_positions.put("arm_servo", 0.25);
                    if (!arm_servo_initialized) {
                        arm_servo_initialized = true;
                    }
                }
            }

            //Set servo positions
            servos.get("slide_servo").setPosition(servo_positions.get("slide_servo"));
            if (arm_servo_initialized) {
                servos.get("arm_servo").setPosition(servo_positions.get("arm_servo"));
            }

            //Set crservo powers
            for (String key : crservos.keySet()) {
                crservos.get(key).setPower(crservo_powers.get(key));
            }

            //Display data on driver hub
            telemetry.addData("Status", "Run Time: " + runtime.toString());
            telemetry.addData("Front left/Right", "%4.2f, %4.2f", wheel_motor_powers.get("front_left"), wheel_motor_powers.get("front_right"));
            telemetry.addData("Back  left/Right", "%4.2f, %4.2f", wheel_motor_powers.get("back_left"), wheel_motor_powers.get("back_right"));
            telemetry.addData("Encoder ticks (fleft)", "%4.2f", motors.get("front_left").getCurrentPosition()/CPR);
            telemetry.addData("Encoder ticks (fright)", "%4.2f", motors.get("front_right").getCurrentPosition()/CPR);
            telemetry.addData("Encoder ticks (bleft)", "%4.2f", motors.get("back_left").getCurrentPosition()/CPR);
            telemetry.addData("Encoder ticks (bright)", "%4.2f", motors.get("back_right").getCurrentPosition()/CPR);
            telemetry.update();
        }
    }
}