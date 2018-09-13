package org.firstinspires.ftc.teamcode.Template;

import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by Simon on 12/09/2018.
 */

public class Auto extends Drive{

    //declar varables
    double EncoderFull = 1440f;
    double DiameterCM = 10.5f;
    double DiameterRobotCM = 37f;

    double circumference;
    double circumferenceRobot;
    double OneDeg;
    double Step;

    DcMotor LeftMotor;
    DcMotor RightMotor;

    public void Init(DcMotor Left,DcMotor Right){
        LeftMotor = Left;
        RightMotor = Right;

        circumference = (double) (2*DiameterCM*Math.PI);
        circumferenceRobot = (double) (2*DiameterRobotCM*Math.PI);
        OneDeg = circumferenceRobot/360;
        Step = circumference/EncoderFull;
    }


    public boolean Forward(float distance){
        LeftMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RightMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        distance = Math.round(distance/(float)Step);
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

    public void SetMotors(float left,float Right){
        LeftMotor.setPower(left);
        RightMotor.setPower(Right);
    }
}
