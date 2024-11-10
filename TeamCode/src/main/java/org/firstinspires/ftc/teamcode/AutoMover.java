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

import static java.lang.Math.abs;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

//NOTE: ALL DISTANCES ARE IN CENTIMETERS, ALL TIMES ARE IN SECONDS
public class AutoMover {
    //Create the variables for the motors and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left_motor = null;
    private DcMotor back_left_motor = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_right_motor = null;

    //These variables can be manually adjusted if certain engines are slightly more powerful than others
    private final double front_left_motor_coefficient = 1;
    private final double back_left_motor_coefficient = 1;
    private final double front_right_motor_coefficient = 1;
    private final double back_right_motor_coefficient = 1;

    //You multiply the distance you want to move (or degrees you want to rotate) by these values to get a time in seconds you have to let the engines run for
    private final double forward_distance_time_conv_fact = 0.01;
    private final double backward_distance_time_conv_fact = 0.01;
    private final double rotation_time_conv_fact = 0.01;

    //Global speed percentage for all movement
    private final double speed_coefficient = 0.1;

    public AutoMover(DcMotor front_left_motor, DcMotor back_left_motor, DcMotor front_right_motor, DcMotor back_right_motor) {
        this.front_left_motor = front_left_motor;
        this.front_right_motor = front_right_motor;
        this.back_left_motor = back_left_motor;
        this.back_right_motor = back_right_motor;

        //Set motor directions
        this.front_left_motor.setDirection(DcMotor.Direction.REVERSE);
        this.back_left_motor.setDirection(DcMotor.Direction.REVERSE);
        this.front_right_motor.setDirection(DcMotor.Direction.FORWARD);
        this.back_right_motor.setDirection(DcMotor.Direction.FORWARD);
    }

    public double move_forward(double distance) {
        //How long the engines should run for
        double running_time = distance*forward_distance_time_conv_fact;

        //Get current time in seconds
        double start = runtime.seconds();

        //Send power to engines
        front_left_motor.setPower(front_left_motor_coefficient*speed_coefficient);
        front_right_motor.setPower(front_right_motor_coefficient*speed_coefficient);
        back_left_motor.setPower(back_left_motor_coefficient*speed_coefficient);
        back_right_motor.setPower(back_right_motor_coefficient*speed_coefficient);

        //Wait until done run time expired
        while (runtime.seconds() > (start + running_time)) {
            //Do nothing
        }

        //Stop engines
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);

        //Used for telemetry and calibration purposes
        return running_time;
    }

    public double move_backward(double distance) {
        //How long the engines should run for
        double running_time = distance*backward_distance_time_conv_fact;

        //Get current time in seconds
        double start = runtime.seconds();

        //Send power to engines
        //Note the negative signs
        front_left_motor.setPower(-front_left_motor_coefficient*speed_coefficient);
        front_right_motor.setPower(-front_right_motor_coefficient*speed_coefficient);
        back_left_motor.setPower(-back_left_motor_coefficient*speed_coefficient);
        back_right_motor.setPower(-back_right_motor_coefficient*speed_coefficient);

        //Wait until done run time expired
        while (runtime.seconds() > (start + running_time)) {
            //Do nothing
        }

        //Stop engines
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);

        //Used for telemetry and calibration purposes
        return running_time;
    }

    //0 degrees is directly forward. To the left is negative
    public double rotate(double degrees) {
        double running_time = abs(degrees)*rotation_time_conv_fact;

        //Get current time in seconds
        double start = runtime.seconds();

        //Send power to engines. Note the sign changes
        front_left_motor.setPower(front_left_motor_coefficient*speed_coefficient);
        front_right_motor.setPower(-front_right_motor_coefficient*speed_coefficient);
        back_left_motor.setPower(back_left_motor_coefficient*speed_coefficient);
        back_right_motor.setPower(-back_right_motor_coefficient*speed_coefficient);

        //Wait until done run time expired
        while (runtime.seconds() > (start + running_time)) {
            //Do nothing
        }

        //Stop engines
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);

        //Used for telemetry and calibration purposes
        return running_time;
    }
}