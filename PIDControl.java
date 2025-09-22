package org.firstinspires.ftc.teamcode;


import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.controller.PIDController;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp(name = "PIDControl")

public class PIDControl extends LinearOpMode {


    //Importing the PID controller as a variable with the name "controller"
    private PIDController controller;

    //Moves the motor to a specific position
    public static double p = 0, i = 0, d = 0;

    //F variable keeps the motor stable under gravity
    public static double f=0;

    //Target position of the motor
    public static int target = 0;

    //Number/360 is ticksperrev/360
    private final double ticks_in_degrees = 1425.1/360;

    //The name after the "DcMotorEx" is the name of the motor
    private DcMotorEx motor;


    @Override
    public void runOpMode () throws InterruptedException{

        //Telling the PIDController which variables to use
        controller = new PIDController(p,i,d);

        //Setting telemetry for FTC dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        //"Motor" should correspond to the name in configuration on driver hub
        motor  = hardwareMap.get(DcMotorEx.class, "motor");



        waitForStart();

        if (isStopRequested()) return;

        while (opModeIsActive()){

            //Redeclaring the PID values
            controller.setPID(p,i,d);
            int motorPos = motor.getCurrentPosition();
            //Weird background math shit below
            double pid = controller.calculate(motorPos, target);
            //Background calculation to hold the motor at a position
            double ff = Math.cos(Math.toRadians(target/ticks_in_degrees));

            //Setting the power for the motor
            double power = pid + ff;


            if (gamepad2.y){
                target = 100;
            }


            motor.setPower(power);
            //Position telemetry for FTC Dashboard
            telemetry.addData("Pos ", motorPos);
            telemetry.addData("target ", target);
            telemetry.update();


        }

    }



}

