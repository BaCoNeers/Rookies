package org.firstinspires.ftc.teamcode.CoolCodes;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="HolmesCoolCode",  group="Holmes")
public class HolmesCoolCode extends OpMode {
    public final String REPLACE_CHAR = "%";

    public final String MOTOR_NAME = "motor%";

    public final int NUMBER_OF_MOTORS = 4;

    public final Float DEADZONE_MIN = 0.06f;

    // zero is empty
    public DcMotor Motor[] = new DcMotor[NUMBER_OF_MOTORS + 1];

    private Float rotation;

    private Float deltaX;

    private Float deltaY;

    private Float power;

    private Float X;

    private Float Y;

    public void init() {
        for (int i = 1; i <= NUMBER_OF_MOTORS; i++)
            Motor[i] = hardwareMap.dcMotor.get(MOTOR_NAME.replace(REPLACE_CHAR, String.valueOf(i)));
    }

    public void loop() {
        power = gamepad1.right_trigger;

        deltaX = (gamepad1.left_stick_x * power) * 0.88f;

        deltaY = (gamepad1.left_stick_y * power) * 0.88f;

        rotation = gamepad1.right_stick_x * 0.88f;

        if ((gamepad1.right_stick_x < 0 ? -gamepad1.right_stick_x : gamepad1.right_stick_x) < DEADZONE_MIN)
            rotation = 0f;

        if ((gamepad1.left_stick_x < 0 ? -gamepad1.left_stick_x : gamepad1.left_stick_x) < DEADZONE_MIN)
            deltaX = 0f;

        if ((gamepad1.left_stick_y < 0 ? -gamepad1.left_stick_y : gamepad1.left_stick_y) < DEADZONE_MIN)
            deltaY = 0f;

        Y = ((deltaY + rotation) <= 1 ? (deltaY + rotation) : 1);

        X = ((deltaX + rotation) <= 1 ? (deltaX + rotation) : 1);

        // vertical
        Motor[1].setPower(-Y);
        Motor[4].setPower(rotation != 0 ? Y * -1 : Y);

        // horizontal
        Motor[2].setPower(-X);
        Motor[3].setPower(rotation != 0 ? X * -1 : X);

        //test code
        /*if (gamepad1.left_trigger> 0.8f) {

            Motor[2].setPower(-X* (1-gamepad1.left_trigger));

        }*/
    }}
