
package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
@TeleOp(name = "PIDTest", group = "A1")

public class PID_Config extends LinearOpMode {

    private PIDController controller;
    public static double p = 0, i = 0, d = 0;
    public static double f = 0;



    public static int target = 0;
    //public int target;



    public static int target1;

    private final double ticks_in_degrees = 1425.1/360.0; //change the 360 back to 180 if no work

    private DcMotorEx motor ;




    @Override
    public void runOpMode() throws InterruptedException {



        //set target to zero here

        controller = new PIDController(p,i,d);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        motor = hardwareMap.get(DcMotorEx.class, "Arm");





        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()) {


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
            controller.setPID(p,i,d);
            int armPos = motor.getCurrentPosition();
            double pid = controller.calculate(armPos, target);
            double ff = Math.cos(Math.toRadians(target/ticks_in_degrees)) * f;

            double power = pid + ff;

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////






            telemetry.addData("pos ", armPos);
            telemetry.addData("target ", target);
            telemetry.update();



        }
    }
}