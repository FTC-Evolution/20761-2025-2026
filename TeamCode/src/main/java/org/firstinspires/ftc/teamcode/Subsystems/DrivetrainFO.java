package org.firstinspires.ftc.teamcode.Subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.IMU;


public class DrivetrainFO {
    private DcMotor frontRight;     // Creating all object
    private DcMotor frontLeft;
    private DcMotor rearRight;
    private DcMotor rearLeft;
    private IMU imu;
    private  boolean _state;
    double cm = (569.6)/(9.6 * 3.14);
    int cm_entier = (int)cm;



    public DrivetrainFO(DcMotor frontLeft, DcMotor frontRight, DcMotor rearLeft, DcMotor rearRight, IMU imu){
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.rearLeft = rearLeft;
        this.rearRight = rearRight;
        this.imu = imu;

        frontRight.setDirection(DcMotorSimple.Direction.FORWARD);   // setting the moteur direction
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        rearRight.setDirection(DcMotorSimple.Direction.FORWARD);
        rearLeft.setDirection(DcMotorSimple.Direction.REVERSE);

    }
    public boolean slowMode = false;
    public void switchState(){
        setState(!_state);
    }

    public void setState(boolean state){
        _state = state;
        if (_state) {
           slowMode = true;
        }else{
            slowMode = false;
        }
    }

    public void mecanumDrive(double rotx, double roty, double r) {    //Fonction for an easy tank drive(taken from FRC )
       if(slowMode) {
           frontLeft.setPower(roty/2 + rotx/2 + r/2);
           frontRight.setPower(roty/2 - rotx/2 - r/2);
           rearLeft.setPower(roty/2 - rotx/2 + r/2);
           rearRight.setPower(roty/2 + rotx/2 -r/2);
       } else {
           frontLeft.setPower(roty + rotx + r);
           frontRight.setPower(roty - rotx - r);
           rearLeft.setPower(roty - rotx + r);
           rearRight.setPower(roty + rotx - r);
       }

    }

    public int getFrontRightPosition(){
        return frontRight.getCurrentPosition();
    }

    public int getFrontLeftPosition(){
        return frontLeft.getCurrentPosition();
    }

    public int getRearRightPosition(){
        return rearRight.getCurrentPosition();
    }

    public int getRearLeftPosition(){
        return rearLeft.getCurrentPosition();
    }

    public void resetEncoders(){
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    }

    public void setFrontRightPosition(double speed, int position){
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setTargetPosition(position);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanumDrive(0, speed, 0);
    }
    public void setFrontLeftPosition(double speed, int position){
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setTargetPosition(position);
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
               mecanumDrive(0, speed, 0);

    }
    public void setRearRightPosition(double speed, int position){
        rearRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearRight.setTargetPosition(position);
        rearRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanumDrive(0, speed, 0);

    }
    public void setRearLeftPosition(double speed, int position){
        rearLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rearLeft.setTargetPosition(position);
        rearLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        mecanumDrive(0, speed, 0);

    }

    public void setAllPosition(double speed, double positionRight, double positionLeft){
        setFrontRightPosition(speed, (int) positionRight*15);
        setFrontLeftPosition(speed, (int) positionLeft*15);
        setRearRightPosition(speed, (int) positionRight*15);
        setRearLeftPosition(speed, (int) positionLeft*15);

        while (motorIsBusy()){

        }
        mecanumDrive(0, 0, 0);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setAllPositionSide(double speed, double positionRight, double positionLeft){
        setFrontRightPosition(speed, (int) positionRight*1440);
        setFrontLeftPosition(speed, (int) positionLeft*1440);
        setRearRightPosition(speed, (int) positionLeft*1440);
        setRearLeftPosition(speed, (int) positionRight*1440);
        while (motorIsBusy()){

        }
        mecanumDrive(0, 0, 0);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rearLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    }

    public void setAllPositionCm(double speed, double positionRight, double positionLeft){


        setAllPosition(speed, positionRight*cm_entier, positionLeft*cm_entier);
    }


    public boolean motorIsBusy(){

        if (frontLeft.isBusy()|| frontRight.isBusy() || rearLeft.isBusy() || rearRight.isBusy()){
            return true;
        }
        return false;
    }


}
