/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

// Dennis Terry
package org.usfirst.frc.team1102.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.*;

import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import com.ctre.phoenix.motorcontrol.ControlMode;

import edu.wpi.cscore.CvSink;
import edu.wpi.cscore.CvSource;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.SampleRobot;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Encoder;
/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
@SuppressWarnings("deprecation")
public class Robot extends SampleRobot {

    VictorSP Winch    = new VictorSP(4);
    VictorSP Wrist    = new VictorSP(8);
   
    TalonSRX Elevator = new TalonSRX(1);
    
    Compressor compressor = new Compressor();
    
    StringBuilder sb = new StringBuilder();

    Encoder LeftEncoder; 
    Encoder RightEncoder;
        
    DigitalInput BotLimit = new DigitalInput(4);
    DigitalInput TopLimit = new DigitalInput(5);
    
    RazerController Joy1 = new RazerController(0);
    LogitechGamepad Joy2 = new LogitechGamepad(1);
    
    IntakeClaw intakeclaw = new IntakeClaw(5,0,1);  
    Drivetrain drive = new Drivetrain(0,1,2,3,0,1,2,3); //REAL ROBOT & Practice Base
    
	private static final String LeftAuto   = "Left Starting Position";
	private static final String CenterAuto = "Center Original Version";
	private static final String CenterAutoDiag1 = "Center Start Diagonal 1 Cube";
	private static final String CenterAutoDiag2 = "Center Start Diagonal 2 Cubes";
	private static final String RightAuto  = "Right Starting Position";
	private static final String LeftScale  = "Left Scale Auto";
	private static final String RightScale = "Right Scale Auto";
	private static final String LeftScaleAutoCross  = "Left Scale Auto Cross";
	private static final String RightScaleAutoCross = "Right Scale Auto Cross";
	
	private String m_autoSelected;  // Prevent runaway
	private SendableChooser<String> m_chooser = new SendableChooser<>();
    
    static final double AutoMax             = 1.00; // Needed for Diagonal
    static final double AutoFast            = 0.75;
    static final double AutoSlow            = 0.40;
    static final double TestFast            = 0.75;
    static final double TestSlow            = 0.40;
    static final double SwitchDistance      = 130.0; // Not including initial 10"
//    static final double ScaleDistance       =  80.0; // Shortened it for testing
    static final double ScaleDistance       = 250.0; // Proper Distance
//    static final double PathwayDistance     = 130.0;// Shortened it for testing
    static final double PathwayDistance     = 240.0; // Proper Distance
      
    double ElevatorPosition;
	boolean failsafe = false;
    boolean StopShort = true; 
	boolean _lastButton1 = false;
	boolean TopLim = false;
	int _loops = 0;
	
    String GameData = "";
    
    //************************************************************************//
    //*                                                                      *//
    //*  Update Dashboard Routine called by the various sections of code     *//
    //*                                                                      *//
    //************************************************************************//
     void UpdateDashboard(){  //* Routine to update the Dashboard 

         SmartDashboard.putNumber("Elevator Position", ElevatorPosition);
         SmartDashboard.putNumber("Elevator Setpoint", LiftSetpoint);		
         SmartDashboard.putNumber("LeftEncoder", drive.LeftEncoder.get());
         SmartDashboard.putNumber("RightEncoder", drive.RightEncoder.get());        
         SmartDashboard.putBoolean("Top Limit", TopLimit.get());
         SmartDashboard.putBoolean("Bottom Limit", BotLimit.get());
         SmartDashboard.putBoolean("IS AUTO?", isAutonomous());
      }
    //*************************************************************************//
 	//*                                                                       *//
 	//* PID Task sets the power to the Lift Drive Motors to keep lift in place*//
 	//*  It looks at the current position and determines power to move/keep   *//
 	//*  lift where you want it                                                *//
 	//*                                                                       *//
 	//*************************************************************************//
     double LiftPower = 0.0;
     double LiftkP = 0.1;  //* Use 45 percent of the difference for the power *//
     double LiftkI = 0.0;
     double LiftkD = 0.0;
     boolean StopPID = false;
     double LIFTFLOOR   = 000;
     double LIFTSTOW    = 100;
     double LIFTSWITCH  = 200;
     double LIFTSCALELO = 300;
     double LIFTSCALEMID= 400;
     double LIFTSCALEHI = 500;
     double LIFTSCALEMAX= 600;
     double LiftSetpoint = LIFTFLOOR;    //* Starting point is the arm being down *//
     double LiftError = 0;
     double LiftIntegral = 0;
     double LiftPrevError = 0;
     double LiftDerivative = 0;
     
     public void disabled() {
     	while(isDisabled()) {
    		SmartDashboard.putData("Auto Choices", m_chooser);
    		drive.setAuto(false);
     	}
     } 
     //*************************************************************************//
     //*                                                                       *//
     //*    Defines Drive Ratio versus Motor Speed, Wheel Size,                *//
     //*    Encoder Counts per motor rev. Arithmetic gives us Encoder          *//
     //*    Counts per inch. Counts per degree is trial and error              *//
     //*    Counts per second is used to develop timeout values using the      *//
     //*    lowest motor speed @ full torque at full power.                    *//
     //*                                                                       *//
     //*************************************************************************//
       static final int    GO    = 1;         //* Type of Move,Forward or Back *//
       static final int    SPIN  = 2;         //* Type of Move, Both sides Opp *//
       static final int    SWEEP = 3;         //* Type of Move, Strafe         *//
       static final int    PIVOTL= 4;         //* Type of Pivot - Left fixed   *//
       static final int    PIVOTR= 5;         //* Type of Pivot - Right Fixed  *//
       static final int    DIAGF = 6;         //* Go diagonal Forward          *//
       static final int    DIAGB = 7;         //* Go diagonal Backward         *//
  
   //*************************************************************************//
   //*  Open the Claw                                                        *//  
   //*************************************************************************//
   void OpenClaw() {
	  if(!isAutonomous())
		  return;
      intakeclaw.setClaw(Value.kForward);
      Timer.delay(0.4);
   }
   //*************************************************************************//
   //*  Close the Claw                                                      *//  
   //*************************************************************************//
   void CloseClaw() {
      if(!isAutonomous())
		  return;
      intakeclaw.setClaw(Value.kReverse);
      Timer.delay(0.4);
   }
   //*************************************************************************//
   //*  Drop the Wrist                                                       *//  
   //*************************************************************************//
   void DropWrist() {
	  if(!isAutonomous())
		  return;
       Wrist.set(0.5);
       Timer.delay(0.4);
       Wrist.set(0);
   } 
   //*************************************************************************//
   //*  Raise the Wrist                                                       *//  
   //*************************************************************************//
   void RaiseWrist() {
	if(!isAutonomous())
		return;
       Wrist.set(-0.5);
       Timer.delay(0.4);
       Wrist.set(0);
   } 
   //*************************************************************************//
   //*  Grab the Cube                                                        *//  
   //*************************************************************************//
   void GrabCube() {
		  if(!isAutonomous())
			  return;
	  intakeclaw.setWheels(1);  // Wheels running in
 	  Timer.delay(0.2);
      intakeclaw.setClaw(Value.kForward); // Close the Claw
 	  Timer.delay(0.2);
      intakeclaw.setWheels(0); // Stop the Wheels
      //Timer.delay(0.3);
   } 
   //*************************************************************************//
   //*  Spit Out the Cube                                                    *//  
   //*************************************************************************//
   void SpitOutCube() {
		  if(!isAutonomous())
			  return;
 	   intakeclaw.setWheels(2); // Eject quickly wheels running fast
 	   Timer.delay(0.25);
       intakeclaw.setWheels(0); // Stop the wheels
       //intakeclaw.setClaw(Value.kReverse); //Open the Claw
   }
   //*************************************************************************//
   //*  Drop the Cube                                                        *//  
   //*************************************************************************//
   void DropCube() {
		  if(!isAutonomous())
			  return;
       intakeclaw.setWheels(2); // Wheels running out slowly
 	   Timer.delay(0.2);
       intakeclaw.setClaw(Value.kReverse); // Open the claw
       Timer.delay(0.2);
       intakeclaw.setWheels(0); // Stop the wheels
 	   //Timer.delay(0.5);
   }
   //*************************************************************************//
   //*  Prepare for Telop - Drops Wrist and Lowers the lift                  *//  
   //*************************************************************************//
   void PrepLiftForTelop() {
		  if(!isAutonomous())
			  return;
   	try{
   		new Thread(() -> {
       		while(ElevatorPosition > 0 && isAutonomous() && BotLimit.get() == false) {
       			Elevator.set(ControlMode.PercentOutput, 1.0);
       		}
       		//Elevator.set(ControlMode.PercentOutput, 0);
       		Elevator.set(ControlMode.PercentOutput, 0.5);
       		Timer.delay(0.25);
       		Elevator.set(ControlMode.PercentOutput, 0);
       		Thread.currentThread().interrupt();
   		}).start();
   	}catch(Exception e) {}
       intakeclaw.setWheels(1);
       intakeclaw.setClaw(Value.kReverse);// Open the Claw
       //Timer.delay(0.4);
   }
   //*************************************************************************//
   //*  Prepare for Switch - Drops Wrist and Raises lift to right height     *//  
   //*************************************************************************//
   void PrepLiftForSwitch() {
		  if(!isAutonomous())
			  return;
      try{
     		new Thread(() -> {
     			
     	       Wrist.set(0.4);
     	       Timer.delay(0.4);
     	       Wrist.set(0);
     			int count = 0;
     			TopLim = TopLimit.get();
         		while(ElevatorPosition < 14000 && isAutonomous() && !TopLim) {
         			Elevator.set(ControlMode.PercentOutput, -0.75);
         			count++;
         			if(count > 100) {
         				TopLim = TopLimit.get();
         			}
         		}
         		Elevator.set(ControlMode.PercentOutput, 0);
         		Thread.currentThread().interrupt();
     		}).start();
     	}catch(Exception e) {}
   }
   //*************************************************************************//
   //*  Prepare for Scale - Drops Wrist and raises lift                      *//  
   //*************************************************************************//
   void PrepLiftForScale() {
		  if(!isAutonomous())
			  return;
   	try{
 		new Thread(() -> {
 	       Wrist.set(0.6);
 	       Timer.delay(0.2);
 	       Wrist.set(0);
 			int count4 = 0;
 			TopLim = TopLimit.get();
     		while(ElevatorPosition < 34000 && isAutonomous() && !TopLim) {
     			Elevator.set(ControlMode.PercentOutput, -0.9);
     			count4++;
     			if(count4 > 100) {
     				TopLim = TopLimit.get();
     			}
     		}
     		Elevator.set(ControlMode.PercentOutput, 0);
     		
     		Thread.currentThread().interrupt();
 		}).start();
 	        }catch(Exception e) {}
   		
   }
   //*************************************************************************//
   //*  Get Across the Line - Drops the Wrist and goes forward               *//  
   //*************************************************************************//
   void GO_Across_Line() {
		  if(!isAutonomous())
			  return;
       Wrist.set(0.5);
       Timer.delay(0.4);
       Wrist.set(0);
 	   drive.RobotMove(GO, 120.0, AutoFast);  // Get close to switch
   }
   //*************************************************************************//
   //*  Switch Right Routine from Center Single Cube                         *//  
   //*************************************************************************//
   void Center_Switch_Right() {
		  boolean auto_complete = false;
		  while(isAutonomous() && !auto_complete) {
			  if(isAutonomous())
 	   drive.RobotMove(GO, 10.0, AutoSlow); // Get away from the wall
			  if(isAutonomous())
 	   PrepLiftForSwitch();
			  if(isAutonomous())
 	   drive.RobotMove(SPIN, 40.0, AutoFast);// Turn towards switch
			  if(isAutonomous())
 	   drive.RobotMove(GO,55.0, AutoFast);  // Get close to switch
			  if(isAutonomous())
 	   drive.RobotMove(SPIN,-35.0,AutoFast); // Turn towards switch
			  if(isAutonomous())
 	   drive.RobotMove(GO, 50.0, AutoFast);  // Get next to switch
			  if(isAutonomous())
 	   SpitOutCube(); 
			  if(isAutonomous())
       drive.RobotMove(GO,-30, AutoFast);  // Get away from switch
			  if(isAutonomous())
       PrepLiftForTelop();
			  if(isAutonomous())
       drive.RobotMove(SPIN,-80.0, AutoFast);  //Turn toward Cubes
			  if(isAutonomous())
       auto_complete = true;
		  }
   }
   //*************************************************************************//
   //*  Switch Left Routine from Center Single Cube                          *//  
   //*************************************************************************//
   void Center_Switch_Left() {
		  boolean auto_complete = false;
		  while(isAutonomous() && !auto_complete) {
			  if(isAutonomous())
 	   drive.RobotMove(GO, 10.0, AutoSlow);   // Get away from the wall
			  if(isAutonomous())
 	   PrepLiftForSwitch();
			  if(isAutonomous())
       drive.RobotMove(SPIN, -45.0, AutoFast);// Turn towards switch
			  if(isAutonomous())
 	   drive.RobotMove(GO, 70.0, AutoFast);   // Get close to switch
			  if(isAutonomous())
 	   drive.RobotMove(SPIN, 50.0, AutoFast); // Turn towards switch
			  if(isAutonomous())
 	   drive.RobotMove(GO, 36.0, AutoFast);   // Get next to switch
			  if(isAutonomous())
       SpitOutCube();
			  if(isAutonomous())
       drive.RobotMove(GO,-28, AutoSlow);     // Get away from switch
			  if(isAutonomous())
       PrepLiftForTelop();
			  if(isAutonomous())
       drive.RobotMove(SPIN,80.0, AutoFast);  //Turn toward Cubes
		  }
   }
   //*************************************************************************//
   //*  Switch Right Routine using the Diagonal Moves - Up to Double Cube    *//  
   //*************************************************************************//
   void Center_Switch_Right_Diag(int cubes) {
		  boolean auto_complete = false;
		  while(isAutonomous() && !auto_complete) {
	  PrepLiftForSwitch();
  	  drive.RobotPivot(DIAGF,90,1.0);
 	  DropCube();
      PrepLiftForTelop();
      drive.RobotPivot(DIAGB,-70, AutoMax);  // Get away from switch
      drive.RobotMove(GO, 24, AutoFast);
      if (cubes == 1) return;
      GrabCube();
      RaiseWrist();
      drive.RobotPivot(DIAGB, 18, AutoFast);
      PrepLiftForSwitch();
      drive.RobotPivot(DIAGF, 50, AutoMax);
      DropCube();
      drive.RobotMove(GO, -24, AutoFast);
      PrepLiftForTelop();
      auto_complete = true;
		  }
   }
   //*************************************************************************//
   //*  Switch Left Routine using the Diagonal Moves - Up to Double Cube     *//  
   //*************************************************************************//
   void Center_Switch_Left_Diag(int cubes) {
		  boolean auto_complete = false;
		  while(isAutonomous() && !auto_complete) {
	  PrepLiftForSwitch();
	  drive.RobotPivot(DIAGF,-120,AutoMax);
	  DropCube();
	  PrepLiftForTelop();
	  drive.RobotPivot(DIAGB, 120, AutoMax);  // Get away from switch
	  drive.RobotMove(GO, 36, AutoFast);
      if (cubes == 1) return;
	  GrabCube();
	  RaiseWrist();
	  drive.RobotPivot(DIAGB, -24, AutoFast);
	  PrepLiftForSwitch();
	  drive.RobotPivot(DIAGF,-70, AutoMax);
	  DropCube();
	  drive.RobotMove(GO, -24, AutoFast); 
	  PrepLiftForTelop();
	  auto_complete = true;
		  }
   }
   //*************************************************************************//
   //*  Switch Left Routine from Left Side                                   *//  
   //*************************************************************************//
   void Switch_Left() {
		  boolean auto_complete = false;
		  while(isAutonomous() && !auto_complete) {
 	  drive.RobotMove(GO, 10.0, AutoSlow);   // Get away from the wall
 	  PrepLiftForSwitch();
 	  drive.RobotMove(GO,SwitchDistance, AutoFast);   // Get close to switch  		  
 	  drive.RobotMove(SPIN, 90.0, AutoFast); // Turn towards switch
 	  drive.RobotMove(GO, 20.0, AutoSlow);   // Get next to switch
      DropCube();
      drive.RobotMove(GO,-24, AutoSlow);     // Get away from switch
      drive.RobotMove(SPIN,-35.0, AutoFast); // Turn toward cubes
      PrepLiftForTelop();  
      auto_complete = true;
		  }
   }
   //*************************************************************************//
   //*  Switch Right Routine from Right Side                                 *//  
   //*************************************************************************//
   void Switch_Right() {
		  boolean auto_complete = false;
		  while(isAutonomous() && !auto_complete) {
 	  drive.RobotMove(GO, 10.0, AutoSlow); // Get away from the wall
 	  PrepLiftForSwitch();
 	  drive.RobotMove(GO,SwitchDistance, AutoFast); // Get next to the switch
 	  drive.RobotMove(SPIN,-75.0, AutoFast);// Turn towards switch
 	  drive.RobotMove(GO, 6.0, AutoSlow);  // Get close to switch
 	  DropCube();
      drive.RobotMove(GO,-24, AutoFast);  // Get away from switch
      drive.RobotMove(SPIN,45.0, AutoFast);//Turn toward Cubes
      PrepLiftForTelop();
      auto_complete = true;
		  }
   }
   //*************************************************************************//
   //*  Scale Right Routine Passed the number of cubes to do 1 or 2           *//  
   //*************************************************************************//
   void Scale_Right(int cubes) {
		  boolean auto_complete = false;
		  while(isAutonomous() && !auto_complete) {
	  if(isAutonomous())
		  PrepLiftForScale();
	  if(isAutonomous())
		  drive.RobotMove(GO,ScaleDistance,AutoFast); // Go to Scale
	  if(isAutonomous())
		  drive.RobotMove(SPIN,-12.0,AutoSlow); // Turn to scale
      RaiseWrist();
	  if(isAutonomous())
		  SpitOutCube();
	  if(isAutonomous())
		  drive.RobotMove(GO,-20,AutoFast);
	  if(isAutonomous())
		  PrepLiftForTelop();
	  if(isAutonomous())
		  intakeclaw.setClaw(Value.kReverse);
	  if(isAutonomous())
		  drive.RobotMove(SPIN, -96.0,AutoFast); // Turn toward cubes
      
      if (cubes == 1)return;
      if(isAutonomous())
    	  drive.RobotMove(GO, 30.0, AutoFast);  // Go get a cube
      if(isAutonomous()) {
    	  GrabCube();
      	  Timer.delay(0.25);
      }
      if(isAutonomous())
    	  drive.RobotMove(GO,-10.0,AutoFast);
      if(isAutonomous())
    	  PrepLiftForScale();
      if(isAutonomous())
    	  drive.RobotMove(GO,-20.0,AutoFast);
      if(isAutonomous())
    	  drive.RobotMove(SPIN, 80.0,AutoFast); // Turn back toward scale
      if(isAutonomous())
      drive.RobotMove(GO,20,AutoFast);
      if(isAutonomous())
    	  RaiseWrist();
    	  SpitOutCube();
      if(isAutonomous())
    	  drive.RobotMove(GO,-20,AutoFast);
      if(isAutonomous())
    	  drive.RobotMove(SPIN,-80.0,AutoFast); // Turn toward cubes
      if(isAutonomous())
    	  PrepLiftForTelop();
      auto_complete = true;
		  }
      
   }
   //*************************************************************************//
   //*  Scale Left Routine Passed the number of cubes to do 1 or 2           *//  
   //*************************************************************************//
   void Scale_Left(int cubes) {
	  boolean auto_complete = false;
	  while(isAutonomous() && !auto_complete) {
      PrepLiftForScale();
      drive.RobotMove(GO,ScaleDistance,AutoFast); // Go to Scale
      drive.RobotMove(SPIN,10.0,AutoFast); // Turn to scale
      //RaiseWrist();
      SpitOutCube();               // Drop Cube
      drive.RobotMove(GO,-20.0,AutoFast);
      drive.RobotMove(SPIN, 60.0,AutoFast); // Turn toward cubes
      PrepLiftForTelop();
      
	  intakeclaw.setClaw(Value.kReverse);
      if(cubes ==1) return;
      drive.RobotMove(GO, 20.0, AutoSlow);  // Go get a cube
      GrabCube();
      drive.RobotMove(GO,-20.0,AutoFast);
      PrepLiftForScale();
      drive.RobotMove(SPIN,-90.0,AutoFast); // Turn back toward scale
      RaiseWrist();
      SpitOutCube();
      drive.RobotMove(SPIN, 90.0,AutoFast); // Turn toward cubes
      PrepLiftForTelop();
      auto_complete = true;
	  }
   }
   //*************************************************************************//
   //*  Scale Right Crossover - Start on Right side Stop short halts          *//  
   //*************************************************************************//
   void Scale_Right_Crossover(boolean StopShort) {
		  boolean auto_complete = false;
		  while(isAutonomous() && !auto_complete) {
 	 
 	  drive.RobotMove(GO, PathwayDistance,AutoFast); // Go to Scale
 	  drive.RobotMove(SPIN,-90.0,AutoSlow); // Turn toward scale
 	  PrepLiftForScale();
 	  drive.RobotMove(GO, 80.0,AutoFast);   // Go to Scale 
 	  if(StopShort) return;
 	  drive.RobotMove(GO, 80.0,AutoFast);   // Go to Scale 
 	  drive.RobotMove(SPIN, 85.0,AutoFast); // Turn toward Scale Platform
 	  drive.RobotMove(GO, 20.0, AutoSlow);  // Go to scale
 	  SpitOutCube();	 
 	  drive.RobotMove(GO,-20,AutoSlow);     //  Back away from scale
 	  PrepLiftForTelop();
 	  drive.RobotMove(SPIN,-180.0,AutoSlow); // Turn back toward new cube
      auto_complete = true;
	  }
   }
   //*************************************************************************//
   //*  Scale Left Crossover  - Starts on Left Side Stop short halts         *//  
   //*************************************************************************//
   void Scale_Left_Crossover(boolean StopShort) {
		  boolean auto_complete = false;
		  while(isAutonomous() && !auto_complete) {
 	  //DropWrist();
 	  drive.RobotMove(GO, PathwayDistance,AutoFast); // Go to Scale
 	  drive.RobotMove(SPIN,80.0,AutoFast);   // Turn toward scale
 	  PrepLiftForScale();
 	  drive.RobotMove(GO, 80.0,AutoFast);    // Go to Scale 
 	  if(StopShort)return;
 	  drive.RobotMove(GO, 80.0,AutoFast);    // Go to Scale 
 	  drive.RobotMove(SPIN, -85.0,AutoSlow); // Turn toward Scale Platform
 	  drive.RobotMove(GO, 20.0, AutoSlow);   // Go to scale
 	  RaiseWrist();
 	  SpitOutCube();	  
 	  drive.RobotMove(GO,-20,AutoSlow);      //  Back away from scale
 	  PrepLiftForTelop();
 	  drive.RobotMove(SPIN, 180.0,AutoSlow); // Turn back toward new cube
      auto_complete = true;
	  }
   }
   	//*************************************************************************//
    //*                                                                       *//
    //* This function called once each time the robot enters autonomous mode  *//
    //*                                                                       *//
    //*************************************************************************//
    @Override
    public void autonomous() {
    	drive.setAuto(true);
    	
    	UpdateDashboard();
        SmartDashboard.putBoolean("Autonomous Complete ", false);
    	
        intakeclaw.setClaw(Value.kForward); //Close the Claw
		
    	try{
    		new Thread(() -> {
        		while(isAutonomous()) {
        	        UpdateDashboard();
        		}
        		//Elevator.set(ControlMode.PercentOutput, 0);
        		Thread.currentThread().interrupt();
    		}).start();
    	}catch(Exception e) {}
        
        m_autoSelected = m_chooser.getSelected();
        Timer.delay(0.25);
        
        SmartDashboard.putString("Starting Postition Selected: ", m_autoSelected);
        int timeout = 0;
        
        while(GameData.length() < 3 || timeout < 100000) {
        	GameData = DriverStation.getInstance().getGameSpecificMessage();
        	timeout++;
        }
        
        SmartDashboard.putString("Game Data ",GameData);
        
        switch(m_autoSelected){
        	case CenterAuto:
        		if(GameData.equals("RRR") || GameData.equals("RLR")) {
        			Center_Switch_Right();
        		}
        		else {
            		if(GameData.equals("LLL") || GameData.equals("LRL")) {
                        Center_Switch_Left();
            		}
        	   }
        	   break;
        	case CenterAutoDiag1:
        		if(GameData.equals("RRR") || GameData.equals("RLR")) {
        			Center_Switch_Right_Diag(1); // One Cube
        		}
        		else {
            		if(GameData.equals("LLL") || GameData.equals("LRL")) {
                        Center_Switch_Left_Diag(1); // One Cube
            		}
        	   }
        	   break;
        	case CenterAutoDiag2:
        		if(GameData.equals("RRR") || GameData.equals("RLR")) {
        			Center_Switch_Right_Diag(2); // Two Cubes
        		}
        		else {
            		if(GameData.equals("LLL") || GameData.equals("LRL")) {
                        Center_Switch_Left_Diag(2); // Two Cubes
            		}
        	   }
        	   break;
        	case LeftAuto: // If possible does the Switch first then scale
           		if(GameData.equals("RRR")) GO_Across_Line();
           		if(GameData.equals("LLL") || GameData.equals("LRL")) Switch_Left();
           		if(GameData.equals("RLR")) Scale_Left(1);
        	    break;
        	
        	case RightAuto:	// If possible does the switch first then the scale	
          		if(GameData.equals("LLL")) GO_Across_Line();
           		if(GameData.equals("RRR") || GameData.equals("RLR")) Switch_Right();
           		if(GameData.equals("LRL")) Scale_Right(1);
        	    break;
        	
        	case LeftScale:
        		if(GameData.equals("LLL") || GameData.equals("RLR")) {
        		   Scale_Left(2);   
        	    }else if(GameData.equals("LRL")) {
        	    	      Switch_Left();
        	          }else  GO_Across_Line();
        	    break;
        	
        	case RightScale:
        		if(GameData.equals("RRR") || GameData.equals("LRL")) {
        			Scale_Right(2);
        		}else if(GameData.equals("RLR")) {
  	    	              Switch_Right();
  	                 }else  GO_Across_Line();
        	    break;
        	    
        	case LeftScaleAutoCross:
        		if(GameData.equals("LLL") || GameData.equals("RLR")) {
        		   Scale_Left(2);   
        	    }else {
        		   Scale_Left_Crossover(StopShort);
        	    }
        	    break;
        	
        	case RightScaleAutoCross:
        		if(GameData.equals("RRR") || GameData.equals("LRL")) {
        			Scale_Right(2);
        		}else {
         		   Scale_Right_Crossover(StopShort);
        		}
        	    break;
        	default:
        		break;
        }
        SmartDashboard.putBoolean("Autonomous COMPLETE ", true);
    	
    }
    //*************************************************************************//
    //*                                                                       *//
    //* This function is run when the robot is first started up and should be *//
	// * used for any initialization code.                                    *//
    //*                                                                       *//
    //*************************************************************************// 
    @Override
	public void robotInit() {
		m_chooser.addDefault("Center Original Version", CenterAuto);
		m_chooser.addObject("Center Start Diagonal 1 Cube", CenterAutoDiag1);
		m_chooser.addObject("Center Start Diagonal 2 Cubes", CenterAutoDiag2);
		m_chooser.addObject("Left Starting Position", LeftAuto);
		m_chooser.addObject("Right Starting Position", RightAuto);
		m_chooser.addObject("Left Scale ", LeftScale);
		m_chooser.addObject("Right Scale ", RightScale);
		m_chooser.addObject("Left Scale Auto Cross", LeftScaleAutoCross);
		m_chooser.addObject("Right Scale Auto Cross", RightScaleAutoCross);


    	
     	drive.LeftEncoder.reset();
    	drive.RightEncoder.reset();
    	    	
    	try{
    	new Thread(() -> {
    		// Creates UsbCamera and MjpegServer [1] and connects them

    		CameraServer.getInstance().startAutomaticCapture();

    		// Creates the CvSink and connects it to the UsbCamera 
    		CvSink cvSink = CameraServer.getInstance().getVideo();

    		// Creates the CvSource and MjpegServer [2] and connects them 
    		CvSource outputStream = CameraServer.getInstance().putVideo("Blur", 480, 480); 
            
        }).start();
    	}catch(Exception e) {}
    	


		/* choose the sensor and sensor direction */
		Elevator.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Absolute, Constants.kPIDLoopIdx,
				Constants.kTimeoutMs);

		/* choose to ensure sensor is positive when output is positive */
		Elevator.setSensorPhase(Constants.kSensorPhase);

		/* choose based on what direction you want forward/positive to be.
		 * This does not affect sensor phase. */ 
		Elevator.setInverted(Constants.kMotorInvert);

		/* set the peak and nominal outputs, 12V means full */
		Elevator.configNominalOutputForward(0, Constants.kTimeoutMs);
		Elevator.configNominalOutputReverse(0, Constants.kTimeoutMs);
		Elevator.configPeakOutputForward(1, Constants.kTimeoutMs);
		Elevator.configPeakOutputReverse(-1, Constants.kTimeoutMs);
		/*
		 * set the allowable closed-loop error, Closed-Loop output will be
		 * neutral within this range. See Table in Section 17.2.1 for native
		 * units per rotation.
		 */
		Elevator.configAllowableClosedloopError(0, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

		/* set closed loop gains in slot0, typically kF stays zero. */
		Elevator.config_kF(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		Elevator.config_kP(Constants.kPIDLoopIdx, 0.1, Constants.kTimeoutMs);
		Elevator.config_kI(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);
		Elevator.config_kD(Constants.kPIDLoopIdx, 0.0, Constants.kTimeoutMs);

		/*
		 * lets grab the 360 degree position of the MagEncoder's absolute
		 * position, and intitally set the relative sensor to match.
		 */
		int absolutePosition = Elevator.getSensorCollection().getPulseWidthPosition();
		/* mask out overflows, keep bottom 12 bits */
		absolutePosition &= 0xFFF;
		if (Constants.kSensorPhase)
			absolutePosition *= -1;
		if (Constants.kMotorInvert)
			absolutePosition *= -1;
		/* set the quadrature (relative) sensor to match absolute */
		Elevator.setSelectedSensorPosition(absolutePosition, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
		
    	if(BotLimit.get())
    		Elevator.getSensorCollection().setPulseWidthPosition(0, 100);
		
    	try{
    		new Thread(() -> {
    			while(true) {
    				ElevatorPosition = Elevator.getSelectedSensorPosition(0);
    			}
    		}).start();
    	}catch(Exception e) {}
    	
   	
	}

    //*************************************************************************//  
    //                   Telop Init                                           *// 
    //*************************************************************************//
    public void teleopInit() {
    	drive.setAuto(false);
	}

	//*************************************************************************//
	//*                                                                       *//
	//*    This function is called during teleop mode.                        *//
	//*                                                                       *//
	//*************************************************************************// 
    @Override
	public void operatorControl() {
    	drive.setAuto(false);
		// Start the endless loop to drive and operate the Robot. Go Team!!!!
        while(isOperatorControl() && isEnabled()){
        	 drive.setAuto(false);
           SmartDashboard.putNumber("LeftEncoder", drive.LeftEncoder.get());
           SmartDashboard.putNumber("RightEncoder", drive.RightEncoder.get());	
       	compressor.start();
    	
       	commonLoop();
       	
       	if(BotLimit.get())
       		Elevator.getSensorCollection().setPulseWidthPosition(0, 100);
       	
       	if(Joy1.bbutton()) {
       		intakeclaw.setClaw(Value.kForward);
       	}
       	
       	if(Joy1.abutton()) {
       		intakeclaw.setClaw(Value.kReverse);
       	}
       	
       	if(Joy1.rtbutton()) {
       		intakeclaw.setWheels(3); //Eject Cube
       	}
       	else {
       		if(Joy1.ltbutton()) {
       			intakeclaw.setWheels(1); //Intake Cube
       		}
       		else {
       			if(Joy2.ltbutton()) {
       				intakeclaw.setWheels(1); //Intake Cube
       			}else {
       				intakeclaw.setWheels(0);
       			}
       		}      	
       	}
   		   	   
           Wrist.set(Joy2.rightthumby_dmode() * 0.8);
           if(Joy2.rtbutton()) {
           	Winch.set(1.0);
           }else {
           	if(Joy2.rbbutton()) {
           		Winch.set(-1.0);
           	}else {
           		if(Joy2.ybutton()) {
           			Winch.set(0.1);
           		}else {
           			Winch.set(0.0);
           		}
           	}
        }	 
       		drive.MecDrive(Joy1.triggers(), Joy1.leftthumby(), Joy1.rightthumbx()); 	
  		   UpdateDashboard();
       }
       UpdateDashboard();
    }
    
    
    
    
    
    
    
    
    
   
    
    
    
    
    
    
	//*************************************************************************//
	//*       This function is called during test mode.                       *//
	//*************************************************************************// 
    @Override
	public void test() {
		UpdateDashboard();
		while(isEnabled()) {
  	       SmartDashboard.putNumber("LeftEncoder", drive.LeftEncoder.get());
	       SmartDashboard.putNumber("RightEncoder",drive.RightEncoder.get());	        	 
	       drive.MecDrive(Joy2.triggers(), Joy2.leftthumby(), Joy2.rightthumbx());
		   // Check the buttons for Diagonal and switch Testing 
	  	   if(Joy1.backbutton()){
	          Center_Switch_Left_Diag(1);
	       } 
	  	   if(Joy1.ltbutton()){
		       Switch_Left();
		       } 
	  	   if(Joy1.startbutton()){
	          Center_Switch_Right_Diag(1);       
	       } 
	  	   if(Joy1.rtbutton()){
		      Switch_Right();       
		       } 
	       if(Joy1.abutton()) {  //Test Diagonal backward to left 
	          drive.RobotPivot(DIAGB,-90.0,AutoMax);
	       }
	       if(Joy1.bbutton()){  // Test Diagonal backward to right	   
	          drive.RobotPivot(DIAGB, 120.0,AutoMax); 
	       }
	       if(Joy1.xbutton()) { //Test Diagonal forward to left
	   		  drive.RobotPivot(DIAGF,-120,AutoMax); // stop short
	       }
	       if(Joy1.ybutton()) {  //Test Diagonal forward to right 
	     	  drive.RobotPivot(DIAGF,95,AutoMax);
	       }
           // Check the buttons for Move and Pivot Testing 
           if(Joy1.dpadl()){
        	   Center_Switch_Left();
           }
  		   if(Joy1.dpadr()){
     		   Scale_Right(2);
           } 
   		   if(Joy1.dpadf()){
       		   Scale_Left(2);
            } 
           if(Joy1.dpadb()){
               Center_Switch_Right();
           }
		}
    }
	//*************************************************************************//
	//*                                                                       *//
    //*************************************************************************//
	void commonLoop() {
		/* get gamepad axis */
		double leftYstick = Joy2.leftthumby();
		double motorOutput = Elevator.getMotorOutputPercent();
		boolean button1 = Joy2.lbbutton();
		boolean button2 = Joy2.ltbutton();
		// deadband gamepad 
		if (Math.abs(leftYstick) < 0.10) {
			// within 10% of zero
			leftYstick = 0;

		}
		// prepare line to print 
		sb.append("\tout:");
		// cast to int to remove decimal places 
		sb.append((int) (motorOutput * 100));
		sb.append("%"); // perc 

		sb.append("\tpos:");
		sb.append(Elevator.getSelectedSensorPosition(0));
		sb.append("u"); // units 
		
		//ElevatorPosition = Elevator.getSelectedSensorPosition(0);
		// on button1 press enter closed-loop mode on target position

		// on button2 just straight drive 
		
			// Percent voltage mode 
			//if(BotLimit.get() == true && leftYstick < 0 && failsafe == false) {
			//	Elevator.set(ControlMode.PercentOutput, 0);
		//	}else {
		
		
				if(TopLimit.get() == true && leftYstick > 0 && failsafe == false) {
					Elevator.set(ControlMode.PercentOutput, 0);
				}else {
					if (button1) {
						if(ElevatorPosition < 14000) {
							Elevator.set(ControlMode.PercentOutput, -1.0);
						}else {
							Elevator.set(ControlMode.PercentOutput, 0);
						}
					}else {
						Elevator.set(ControlMode.PercentOutput, (-1 * leftYstick));			
					}
				}
			
		
		
		if(Joy2.backbutton_dmode()){
			failsafe = true;
		}
		// if Talon is in position closed-loop, print some more info 
		if (Elevator.getControlMode() == ControlMode.Position) {
			// append more signals to print when in speed mode. 
			sb.append("\terr:");
			sb.append(Elevator.getClosedLoopError(0));
			sb.append("u"); // units

			sb.append("\ttrg:");
			sb.append(LiftSetpoint);
			sb.append("u"); // units
		}
		//
		// print every ten loops, printing too much too fast is generally bad
		// for performance
		//
		if (++_loops >= 10) {
			_loops = 0;
			//System.out.println(sb.toString());
		}
		sb.setLength(0);
		// save button state for on press detect
		_lastButton1 = button1;
	
	}
}