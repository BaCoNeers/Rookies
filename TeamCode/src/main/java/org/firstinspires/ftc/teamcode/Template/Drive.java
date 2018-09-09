/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode.Template;


import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.bosch.JustLoggingAccelerationIntegrator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;


@Autonomous(name = "Template", group = "Temp")

public class Drive extends LinearOpMode{

    //declar varables
    double EncoderFull = 1440f;
    double DiameterCM = 10.5f;
    double DiameterRobotCM = 37f;

    double circumference;
    double circumferenceRobot;
    double OneDeg;
    double distance;

    float LeftMotorPower;
    float RightMotorPower;

    Orientation angle;

    DcMotor LeftMotor;
    DcMotor RightMotor;

    BNO055IMU IMU;

    BNO055IMU.Parameters Parameters = new BNO055IMU.Parameters();





    @Override
    public void runOpMode() {


        /* Initialize the hardware variables.
         * The init() method of the hardware class does all the work here
         */
        LeftMotor = hardwareMap.get(DcMotor.class, "leftdrive");
        RightMotor = hardwareMap.get(DcMotor.class, "rightdrive");

        LeftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        Parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        Parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        Parameters.calibrationDataFile = "BNO055IMUCalibration.json";
        Parameters.accelerationIntegrationAlgorithm = new JustLoggingAccelerationIntegrator();

        IMU = hardwareMap.get(BNO055IMU.class,"IMU");
        IMU.initialize(Parameters);


        circumference = (double) (2*DiameterCM*Math.PI);
        circumferenceRobot = (double) (2*DiameterRobotCM*Math.PI);
        OneDeg = circumferenceRobot/360;
        distance = circumference*EncoderFull;

        // Send telemetry message to signify robot waiting;
        telemetry.addData("Say", "Hello Driver");    //
        telemetry.setAutoClear(false);
        Telemetry.Item LeftEncoder = telemetry.addData("Left Encoder", "%12.3f", 0.0);
        Telemetry.Item RightEncoder = telemetry.addData("Right Encoder: ","%12.3f",0);
        //Telemetry.Item LeftMotorTelemety = telemetry.addData("Left Motor Power: ","%12.3f",0);
        //Telemetry.Item RightMotorTelemetry = telemetry.addData("Right Motor Power: ","%12.3f",0);
        Telemetry.Item IMUAngle = telemetry.addData("IMU Angle: ","%12.3f",0);



        // Wait for the game to start (driver presses PLAY)
        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {



            //LeftEncoder.setValue(LeftMotor.getCurrentPosition());
            //RightEncoder.setValue(RightMotor.getCurrentPosition());
            angle = IMU.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.XYZ, AngleUnit.DEGREES);
            IMUAngle.setValue(angle.thirdAngle);

            Turn(90);


            telemetry.update();
        }

    }

    public boolean Forward(float distance){
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        if((LeftMotor.getCurrentPosition()+RightMotor.getCurrentPosition())/2 < distance){
            SetMotors(0.5f,0.5f);
        }
        else if ((LeftMotor.getCurrentPosition()+RightMotor.getCurrentPosition())/2 > distance){
            SetMotors(-0.5f,-0.5f);
        }
        else{
            SetMotors(0,0);
            return true;
        }
        return false;
    }

    public boolean Turn(float degree){
        if(angle.thirdAngle < degree){
            SetMotors(0.5f,-0.5f);
        }
        else if(angle.thirdAngle > degree){
            SetMotors(-0.5f,0.5f);
        }
        else{
            SetMotors(0,0);
            return true;
        }
        return false;
    }

    public void SetMotors(float Left,float Right){
        //LeftMotorTelemety.setValue(Left);
        //RightMotorTelemetry.setValue(Right);
        LeftMotor.setPower(Left);
        RightMotor.setPower(Right);
    }
}


