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

    private final double front_left_motor_coefficient = 1;
    private final double back_left_motor_coefficient = 1;
    private final double front_right_motor_coefficient = 1;
    private final double back_right_motor_coefficient = 1;

    private final double forward_distance_time_conv_fact = 1;
    private final double backward_distance_time_conv_fact = 1;
    private final double rotation_time_conv_fact = 1;

    private final double global_speed_val = 0.1;

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

    public void move_forward(double distance) {
        //How long the engines should run for
        double running_time = distance*forward_distance_time_conv_fact;

        //Get current time in seconds
        double start = runtime.seconds();

        //Send power to engines
        front_left_motor.setPower(front_left_motor_coefficient*global_speed_val);
        front_right_motor.setPower(front_right_motor_coefficient*global_speed_val);
        back_left_motor.setPower(back_left_motor_coefficient*global_speed_val);
        back_right_motor.setPower(back_right_motor_coefficient*global_speed_val);

        //Wait until done run time expired
        while (runtime.seconds() > (start + running_time)) {
            //Do nothing
        }

        //Stop engines
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
    }

    public void move_backward(double distance) {
        //How long the engines should run for
        double running_time = distance*backward_distance_time_conv_fact;

        //Get current time in seconds
        double start = runtime.seconds();

        //Send power to engines
        //Note the negative signs
        front_left_motor.setPower(-front_left_motor_coefficient*global_speed_val);
        front_right_motor.setPower(-front_right_motor_coefficient*global_speed_val);
        back_left_motor.setPower(-back_left_motor_coefficient*global_speed_val);
        back_right_motor.setPower(-back_right_motor_coefficient*global_speed_val);

        //Wait until done run time expired
        while (runtime.seconds() > (start + running_time)) {
            //Do nothing
        }

        //Stop engines
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
    }

    //TODO Do this
    //0 degrees is directly forward. To the left is negative
    public void rotate(double degrees) {
        /*
        double running_time = abs(degrees)*rotation_time_conv_fact;

        //Get current time in seconds
        double start = runtime.seconds();

        //Send power to engines
        front_left_motor.setPower(front_left_motor_coefficient*global_speed_val);
        front_right_motor.setPower(front_right_motor_coefficient*global_speed_val);
        back_left_motor.setPower(back_left_motor_coefficient*global_speed_val);
        back_right_motor.setPower(back_right_motor_coefficient*global_speed_val);

        //Wait until done run time expired
        while (runtime.seconds() > (start + running_time)) {
            //Do nothing
        }

        //Stop engines
        front_left_motor.setPower(0);
        front_right_motor.setPower(0);
        back_left_motor.setPower(0);
        back_right_motor.setPower(0);
        */
    }
}