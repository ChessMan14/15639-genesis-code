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
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.Rotation;

@Autonomous(name="Autonomous 1", group="Robot")
public class Autonomous1 extends LinearOpMode {

    //Create the variables for the motors and initializes a variable that keeps track of how long the opmode has been running
    private ElapsedTime runtime = new ElapsedTime();
    private DcMotor front_left_motor = null;
    private DcMotor back_left_motor = null;
    private DcMotor front_right_motor = null;
    private DcMotor back_right_motor = null;

    //We have to override this function since it has already been defined in the parent class LinearOpMode
    @Override
    //Throws condition is there for the later wait() command; shouldn't ever be an error though
    public void runOpMode() throws InterruptedException {
        //Map the actual physical motors to the variables. The "device_name" variable is set in the driver hub configuration
        front_left_motor = hardwareMap.get(DcMotor.class, "front_left_motor");
        back_left_motor = hardwareMap.get(DcMotor.class, "back_left_motor");
        front_right_motor = hardwareMap.get(DcMotor.class, "front_right_motor");
        back_right_motor = hardwareMap.get(DcMotor.class, "back_right_motor");

        //This data is displayed on the driver hub console
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //Wait until the the start button is pressed on the driver hub
        waitForStart();

        //Reset runtime var
        runtime.reset();

        //Initialize AutoMover
        AutoMover autoMover = new AutoMover(front_left_motor, back_left_motor, front_right_motor, back_right_motor);

        //The rest of the code makes the robot move

        wait(15);

        //Robot needs to move forward a bit so that it has space to rotate (cm)
        autoMover.move_forward(5);
        //(left/counterclockwise is negative)
        autoMover.rotate(-90);
        autoMover.move_forward(240);
        autoMover.move_backward(240);
        autoMover.rotate(90);

    }
}