//package org.firstinspires.ftc.teamcode.depricated.intake;
//
//import org.firstinspires.ftc.teamcode.base.Robot;
//import org.firstinspires.ftc.teamcode.base.part.LoopedRobotPart;
//import org.firstinspires.ftc.teamcode.other.Utils;
//import org.firstinspires.ftc.teamcode.deprecated.arm.Arm;
//import org.firstinspires.ftc.teamcode.depricated.arm2.Arm2;
//
//public class Intake extends LoopedRobotPart {
//    public boolean intaking = false;
//
//    private IntakePosition presetPosition;
//    public boolean isAutonomous = false;
//    public boolean isAnna = false;
//    private double intakeServoPos = 0;
//    private long intakeServoMoveStartTime;
//    private int intakeServoMoveTime;
//
//    private double intakePower = 0;
//
//    public Intake(Robot robot, IntakeHardware hardware, IntakeSettings settings) {
//        super(robot, hardware, settings);
//    }
//    public Intake(Robot robot){
//        super(robot, new IntakeHardware(), new IntakeSettings());
//    }
//
//
//    ////////
//    //base//
//    ////////
//    public void runIntake(float power){
//        if(Math.abs(power) >= ((IntakeSettings) settings).minInputRegisterVal){
//            //TODO change arm to new version
//            Arm2 a2 = (Arm2) robot.getPartByClass(Arm2.class);
//            if(isAnna && a2.armMotorPos < 10) {
//                if (a2 != null) {
//                    a2.armDown(0);
//                }
//            } else {
//                Arm a = (Arm) robot.getPartByClass(Arm.class);
//                if (a != null)
//                    a.setToAPresetPosition((short) 1);
//                    setIntakeToPreset(IntakePosition.DOWN);
//            }
//
//            ((IntakeHardware) hardware).intakeMotor.setTargetPower(power);
//            intakePower = power;
//            intaking = true;
//        } else {
//            stopIntake();
//        }
//    }
//
//    public void stopIntake(){
//        intaking = false;
//        ((IntakeHardware) hardware).intakeMotor.setTargetPower(0);
//    }
//    public void startIntake(float power){
//        ((IntakeHardware) hardware).intakeMotor.setTargetPower(power);
//    }
//
//
//    /////////
//    //servo//
//    /////////
//    public void setIntakeServoPosition(double position){
//        position = Utils.Math.capDouble(position, ((IntakeSettings) settings).servoMinPos, ((IntakeSettings) settings).servoMaxPos);
//        intakeServoMoveStartTime = System.currentTimeMillis();
//        //TODO track servo move time correctly
//        intakeServoMoveTime = (int)(Math.abs(intakeServoPos - position) / ((IntakeSettings) settings).servoSpeed * 360000);
//        ((IntakeHardware) hardware).intakeServo.setPosition(position);
//        intakeServoPos = position;
//    }
//
//    public boolean intakeServoDoneMoving(){
//        return System.currentTimeMillis() - intakeServoMoveStartTime > intakeServoMoveTime;
//    }
//
//    public void setIntakeServoPosToManual(double position){
//        ((IntakeHardware) hardware).intakeServo.setPosition(position);
//    }
//    ///////////
//    //presets//
//    ///////////
//    //intake
//    public void setIntakeToPreset(IntakePosition preset) {
//        setIntakeServoToPreset(preset);
//    }
//
//    //Servo
//    void setIntakeServoToPreset(IntakePosition preset){
//        if(preset != presetPosition) {
//            stopIntake();
//            settings.runMode = 2;
//            setIntakeServoPosition(((IntakeSettings) settings).intakeServoPresets[preset.value]);
//            presetPosition = preset;
//        }
//    }
//
//
//    //////////
//    //teleOp//
//    //////////
//    void teleOpCode(){
//        IntakePosition preset = (IntakePosition)((IntakeSettings) settings).intakePresetSupplier.get();
//        if(preset != null){
//            setIntakeToPreset(preset);
//            return;
//        }
//        runIntake(((IntakeSettings) settings).intakePowerSupplier.get());
//    }
//
//
//    /////////////////////
//    //LoopedRobotPart Methods//
//    /////////////////////
//    @Override
//    public void onConstruct() {
//
//    }
//
//    @Override
//    public void onInit() {
//
//    }
//
//    @Override
//    public void onStart() {
//
//    }
//
//    @Override
//    public void onPause() {
//        ((IntakeHardware) hardware).intakeMotor.setTargetPower(0);
//        intaking = false;
//    }
//
//    @Override
//    public void onUnpause() {
//
//    }
//
//    @Override
//    public void onRunLoop(short runMode) {
//        if(runMode == 1 && !isAutonomous){
//            teleOpCode();
//        }else if(runMode == 2){
//            if(intakeServoDoneMoving())
//                settings.runMode = 1;
//        }
//    }
//
//    // TODO: 11/8/2021 add telemetry
//    @Override
//    public void onAddTelemetry() {
//        robot.addTelemetry("intake power", intakePower);
//    }
//
//    @Override
//    public void onStop() {
//
//    }
//
//
//    /////////
//    //other//
//    /////////
//    public enum IntakePosition{
//        UP((short) 0),
//        DOWN((short) 1);
//
//        short value;
//
//        IntakePosition(short value){
//            this.value = value;
//        }
//    }
//}
