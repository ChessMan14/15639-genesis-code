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

import java.lang.Math;

import java.util.HashMap;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//NOTE: ALL DISTANCES ARE IN CENTIMETERS, ALL TIMES ARE IN SECONDS
public class AutoMover {
    //Create the variables for the motors and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();

    //Map of all motors
    private HashMap<String, DcMotor> motors = new HashMap<>();
    //Map of wheel motor powers
    private HashMap<String, Double> motor_powers = new HashMap<>();
    private HashMap<String, Double> motor_coeffs = new HashMap<>();

    //You multiply the distance you want to move (or degrees you want to rotate) by these values to get a time in seconds you have to let the engines run for
    private final double distance_time_conv_fact = 0.018825;
    private final double rotation_time_conv_fact = 0.01327131859;

    //Global speed percentage for movement
    private final double wheel_speed_coefficient = 0.35;
    //Global speed percentage for rotation
    private final double rotation_speed_coefficient = 0.5;

    public AutoMover(DcMotor front_left_motor, DcMotor back_left_motor, DcMotor front_right_motor, DcMotor back_right_motor) {
        motors.put("front_left", front_left_motor);
        motors.put("back_left", back_left_motor);
        motors.put("front_right", front_right_motor);
        motors.put("back_right", back_right_motor);

        motor_coeffs.put("front_left", 1.0);
        motor_coeffs.put("back_left", 1.18);
        motor_coeffs.put("front_right", 1.045);
        motor_coeffs.put("back_right", 1.18);

        //Set motor directions
        motors.get("front_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("back_left").setDirection(DcMotor.Direction.REVERSE);
        motors.get("front_right").setDirection(DcMotor.Direction.FORWARD);
        motors.get("back_right").setDirection(DcMotor.Direction.FORWARD);
    }

    public double move(double distance, double degrees) {
        //How long the engines should run for
        double running_time = distance*distance_time_conv_fact;

        //Get current time in seconds
        double start = runtime.seconds();

        //Vertical movement
        double axial = distance*Math.cos(Math.toRadians(degrees));
        //Horizontal movement
        double lateral = distance*Math.sin(Math.toRadians(degrees));

        //Calculate how much power to send to each wheel based on vertical/horizontal movement and rotation
        motor_powers.put("front_left", wheel_speed_coefficient*(axial + lateral));
        motor_powers.put("back_left", wheel_speed_coefficient*(axial - lateral));
        motor_powers.put("front_right", wheel_speed_coefficient*(axial - lateral));
        motor_powers.put("back_right", wheel_speed_coefficient*(axial + lateral));

        //Apply individual wheel settings
        for (String key : motor_powers.keySet()) {
            motor_powers.put(key, motor_powers.get(key)*motor_coeffs.get(key));
        }

        //Find the maximum power being applied to a single wheel
        double max;
        max = Math.max(Math.abs(motor_powers.get("front_left")), Math.abs(motor_powers.get("front_right")));
        max = Math.max(max, Math.abs(motor_powers.get("back_left")));
        max = Math.max(max, Math.abs(motor_powers.get("back_right")));

        //If power > 100%, scale down all the power variables.
        if (max > wheel_speed_coefficient) {
            double final_max = max;
            motor_powers.replaceAll((key, val) -> val/final_max*wheel_speed_coefficient);
        }

        //Wait until done run time expired
        while (runtime.seconds() < (start + running_time)) {
            //Send power to the motors
            for (String key : motors.keySet()) {
                motors.get(key).setPower(motor_powers.get(key));
            }
        }

        //Stop engines
        for (String key : motors.keySet()) {
            motors.get(key).setPower(0);
        }

        //Used for telemetry and calibration purposes
        return running_time;
    }



    //0 degrees is directly forward. To the left is negative
    public double rotate(double degrees) {
        double running_time = Math.abs(degrees)*rotation_time_conv_fact;

        //Get current time in seconds
        double start = runtime.seconds();

        //Calculate how much power to send to each wheel
        if (degrees > 0) {
            motor_powers.put("front_left", rotation_speed_coefficient);
            motor_powers.put("back_left", rotation_speed_coefficient);
            motor_powers.put("front_right", -rotation_speed_coefficient);
            motor_powers.put("back_right", -rotation_speed_coefficient);
        }
        else {
            motor_powers.put("front_left", -rotation_speed_coefficient);
            motor_powers.put("back_left", -rotation_speed_coefficient);
            motor_powers.put("front_right", rotation_speed_coefficient);
            motor_powers.put("back_right", rotation_speed_coefficient);
        }

        //Apply individual wheel settings
        for (String key : motor_powers.keySet()) {
            motor_powers.put(key, motor_powers.get(key)*motor_coeffs.get(key));
        }

        //Find the maximum power being applied to a single wheel
        double max;
        max = Math.max(Math.abs(motor_powers.get("front_left")), Math.abs(motor_powers.get("front_right")));
        max = Math.max(max, Math.abs(motor_powers.get("back_left")));
        max = Math.max(max, Math.abs(motor_powers.get("back_right")));

        //If power > 100%, scale down all the power variables.
        if (max > wheel_speed_coefficient) {
            double final_max = max;
            motor_powers.replaceAll((key, val) -> val/final_max*wheel_speed_coefficient);
        }

        //Wait until done run time expired
        while (runtime.seconds() < (start + running_time)) {
            //Send power to the motors
            for (String key : motors.keySet()) {
                motors.get(key).setPower(motor_powers.get(key));
            }
        }

        //Stop engines
        for (String key : motors.keySet()) {
            motors.get(key).setPower(0);
        }

        //Used for telemetry and calibration purposes
        return running_time;
    }
}
