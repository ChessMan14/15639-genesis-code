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

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;
import com.qualcomm.robotcore.util.ElapsedTime;

//NOTE: ALL DISTANCES ARE IN CENTIMETERS, ALL TIMES ARE IN SECONDS
public class CustomCRServo {
    //Initialize a variable that keeps track of how long the opmode has been running
    ElapsedTime runtime = new ElapsedTime();

    //Constant coefficient for power to degrees per second
    double power_coeff = 0.2;

    //Max and min positions the servo can be in
    double max_pos = 1.0;
    double min_pos = 0.0;

    //Last position the servo was set to
    double last_pos = 0.0;

    //Current pos
    double new_pos;

    //Elapsed time when the position was lat updated
    double last_time = 0;

    //Current power as set by setPower
    double current_power = 0;

    //The servo that the instance controls
    Servo base_servo;

    //Initializer
    public CustomCRServo(Servo base_servo) {
        this.base_servo = base_servo;
    }

    //Set the max and min positions for the servo
    public void setMinMax(double min, double max) {
        if (min > Servo.MIN_POSITION) {
            min_pos = min;
        }
        if (max < Servo.MAX_POSITION) {
            max_pos = max;
        }
    }

    //Custom method equivalent to that of setPower for normal CRServos
    public void setPower(double power) {
        if (power > 1.0) {
            current_power = 1.0;
        }
        else {
            current_power = power;
        }

        if (runtime.seconds() > (last_time + 0.1)) {
            last_time = runtime.seconds();
        }
    }

    //Update servo position according to power
    //This needs to be called at least 10 times per second to work properly
    public void update() {
        //Enough time has passed, update pos
        if (runtime.seconds() >= (last_time + 0.1)) {
            new_pos = last_pos + current_power*power_coeff;

            //Make sure new_pos is within limits
            if ((new_pos > min_pos) && (new_pos < max_pos)) {
                last_pos = new_pos;
            }
            else if (new_pos < min_pos) {
                last_pos = min_pos;
            }
            else {
                last_pos = max_pos;
            }
            base_servo.setPosition(new_pos);
            last_time = runtime.seconds();
        }
    }
}