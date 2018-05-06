package org.usfirst.frc.team1102.robot;

import edu.wpi.first.wpilibj.VictorSP;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
// Dennis Terry
public class Drivetrain {
	
	public Drivetrain(int LeftFront1, int LeftRear2, int RightRear2, int RightFront1, int EncoderLeft1, int EncoderLeft2, int EncoderRight1, int EncoderRight2) {
		LeftFront = new VictorSP(LeftFront1);
		LeftRear = new VictorSP(LeftRear2);
		RightFront = new VictorSP(RightFront1);
		RightRear = new VictorSP(RightRear2);
		
		LeftEncoder  = new Encoder(EncoderLeft1, EncoderLeft2,false,Encoder.EncodingType.k4X);
		RightEncoder = new Encoder(EncoderRight1, EncoderRight2,false,Encoder.EncodingType.k4X);
		
		//auto_state = auto;	
	}
	
	VictorSP LeftFront;
	VictorSP LeftRear;
	VictorSP RightFront;
	VictorSP RightRear;
	
	Encoder LeftEncoder;
	Encoder RightEncoder;
	
	double CountLeft;
	double CountRight;
	
	boolean auto_state = false;
	
    double LeftFrontDriveValue, RightFrontDriveValue;
    double LeftRearDriveValue,  RightRearDriveValue;
	  		  
public void MecDrive(double x, double x2, double y)
	 {
		  double FR  =(y +x2 + x);//*max_vel; 
		  double BR  =(y +x2 - x);//*max_vel; 
		  double FL  =(y -x2 + x);//*max_vel; 
		  double BL  =(y -x2 - x);//*max_vel;
		  
	      RightFront.set(FR);
	      LeftFront.set(FL);
	      RightRear.set(BR);
	      LeftRear.set(BL);
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
  static final int    MINLOOPCOUNT= 100; //* Minimum Loop Count allowed   *//
  static final double MINMOVE     = 4.0; //* Minimum move amount 4"       *//
  static final double MINSPIN     = 5.0; //* Minimum spin amount 5 Degrees*//
  static final double MINSWEEP    = 6.0; //* Minimum sweep amount 6"      *//
  static final double MINDIAG     = 6.0; //* Minimum sweep amount 6"      *//
  static final double MINPIVOT    = 5.0; //* Minimum Pivot 5 degrees      *//
  static final double MAXPIDPOWER = .85; //* Max power allowed for PID    *//
  static final double MAXPOWER   = 1.00; //* Max power allowed            *//
  static final double MINPOWER   = 0.20; //* Minimum move power required  *//
  static final double SAMPLERATE = 0.05; //* 20 Times per second          *//
  static final double kP =   0.5;        //* Used to keep slave on track  *//
  static final double kI =   0.0;        //* Used for PID                 *//
  static final double kD =   0.0;        //* Used for PID                 *//
  static final double WHEEL_CIRCUM = 18.85;//* Wheel Circumference-inches *//
  static final double ENC_COUNTS = 90.0;  //* Counts per rev Quad Encoder *//
  static final double COUNTS_PER_INCH=ENC_COUNTS/WHEEL_CIRCUM; //* 4.77   *//
  static final double COUNTS_PER_SECOND = 471.0; //* At Max Speed         *//
  static final double SPIN_COUNTS_PER_DEGREE_LEFT  = 1.75;//* Increment   *//
  static final double SPIN_COUNTS_PER_DEGREE_RIGHT = 1.80;//* Increment   *// 
  static final double SWEEP_COUNTS_PER_INCH_LEFT   = 8.0;//* Increment    *//
  static final double SWEEP_COUNTS_PER_INCH_RIGHT  = 8.0;//* Increment    *//
  static final double DIAG_COUNTS_PER_INCH_LEFT    = 8.1;//* Increment    *//
  static final double DIAG_COUNTS_PER_INCH_RIGHT   = 8.1;//* Increment    *//
  static final double PIVOT_COUNTS_PER_DEGREE_LEFT = 4.0;//* Increment    *//
  static final double PIVOT_COUNTS_PER_DEGREE_RIGHT= 4.0;//* Increment    *//
  static final double MILESTONE_STEP_1             = .60;//* First Step   *//
  static final double MILESTONE_STEP_2             = .70;//* Second Step  *//
  static final double REDUCE_SPEED_STEP_1          = .60;//* First Step   *//
  static final double REDUCE_SPEED_STEP_2          = .50;//* Second Step  *//
//*************************************************************************//
//*   Move Routine: Caller supplies Type (GO/SPIN/SWEEP) ,                *//
//*   distance/degrees/distance +(Forward/Clockwise/Right) or -(Backward) *//
//*          and actual max power (0.2 to 1.0) to be used                 *//
//*  Left Side drive is considered master, will make right motors follow  *//
//*************************************************************************//
void RobotMove(int type, double amount, double power)
{
	System.out.println(auto_state);
	  if(auto_state == false)
		  return;
    int    IntCounts, Totalcounts;   //* Encoder values for moves         *//
    int    Timeout, LoopCounter;     //* Calculate value for timeout      *//
    int    MasterEncoder, SlaveEncoder; //* Encoder values                *//
    int    PartialCounts1, PartialCounts2;//*Partial count to reduce speed*//
    int    Error      = 0;          //* Used for PID                      *//
    double FactorLF   = -1.0;       //* Motor Direction,Left Side Reversed*//
    double FactorLR   = -1.0;       //* Motor Direction,Left Side Reversed*//
    double FactorRF   =  1.0;       //* Motor Direction                   *//
    double FactorRR   =  1.0;       //* Motor Direction                   *//
    double SlavePower, MasterPower, Counts; //* Motor Power and Counts    *//
    double Adjust = 0.5;  //* Adjust factor for Soft start & slowing down *//
    if (power < MINPOWER) return;
    MasterPower = power;            //* Use given power                   *// 
    if(MasterPower > MAXPIDPOWER) MasterPower = MAXPIDPOWER;//* For PID   *//
    SlavePower  =  MasterPower;     //*Already converted to actual power  *//
    switch (type) {
  	  case(GO):
  		 if (Math.abs(amount) < MINMOVE)return;   //* Must move enough  *//
           if (amount < 0) {
              FactorLF =  1.0;      //* Reverse all of the motors         *//
              FactorLR =  1.0;
              FactorRF = -1.0;
              FactorRR = -1.0;
              Counts = -amount * COUNTS_PER_INCH;  //* Backward move     *//
          }else {
          	Counts = amount * COUNTS_PER_INCH;   //* Forward move      *//
          }
  	    break;
  	  case (SPIN):      //* Must be a SPIN Type then                    *//
   		 if (Math.abs(amount) < MINSPIN)return;   //* Must move enough  *//
            if(amount < 0) {  //* Spin to left                            *//
               Counts   = -amount * SPIN_COUNTS_PER_DEGREE_LEFT;
               FactorLF = 1.0;    //* Reverse left side motors            *//
               FactorLR = 1.0;
             }else{               //* Spin to Right                       *//
               Counts   = amount * SPIN_COUNTS_PER_DEGREE_RIGHT;
               FactorRF = -1.0;   //* Reverse Right side motors           *//
               FactorRR = -1.0;
             }
  	       break;
  	  case (SWEEP):             //* Sweep to left or right              *//
   		  if (Math.abs(amount) < MINSWEEP)return; //* Must move enough  *//
            if(amount < 0) {      //* Sweep to left                       *//
          	 Counts = -amount * SWEEP_COUNTS_PER_INCH_LEFT;
               FactorLF =  1.0;   //* Reverse LF & RR motors              *//
               FactorRR = -1.0;
             }else{               //* Sweep to Right                      *//
          	  Counts = amount * SWEEP_COUNTS_PER_INCH_RIGHT;
                FactorLR =  1.0;  //* Reverse LR & RF motors              *//
                FactorRF = -1.0;
             }
  	      break;
  	  default:  
  		  return;    // Unknown type of move
    }
    PartialCounts2 =(int)(Counts * MILESTONE_STEP_2); //* @ 70% slow it   *//
    PartialCounts1 =(int)(Counts * MILESTONE_STEP_1); //* @ 60% slow it   *//
    Timeout= (int) ((Counts/COUNTS_PER_SECOND) /(MasterPower*SAMPLERATE)); 
    if (Timeout < MINLOOPCOUNT) Timeout = MINLOOPCOUNT; //* Minimum loops *//
    IntCounts = (int) Counts;        //* Integer value for the count      *//
    Totalcounts = 0; LoopCounter = 0; //* Clear starting values           *//
    if((Math.abs(amount) < 20.0)|| (power < 0.5)|| type == SWEEP)Adjust = 1.0;//*Too Small*//
    LeftFrontDriveValue = FactorLF * MasterPower * Adjust; //* Motor Power*//
    RightFrontDriveValue= FactorRF * SlavePower  * Adjust; //* Motor Power*//
    LeftRearDriveValue  = FactorLR * MasterPower * Adjust; //* Motor Power*//
    RightRearDriveValue = FactorRR * SlavePower  * Adjust; //* Motor Power*//
    LeftEncoder.reset();             //* Clear the encoders               *//
    RightEncoder.reset();            //* Clear the encoders               *//
    LeftFront.set(LeftFrontDriveValue);      //* Set Motors Power         *//
    RightFront.set(RightFrontDriveValue);    //* Set Motors Power         *//
    LeftRear.set(LeftRearDriveValue);        //* Set Motors Power         *//
    RightRear.set(RightRearDriveValue);      //* Set Motors Power         *//
    Timer.delay(0.2);                //* Time for motors to get started   *//
    while ((LoopCounter < Timeout) && auto_state == true){
  	  if(auto_state == false)
		  return;
//* This waits for motors to reach desired position and then jumps out    *//
      MasterEncoder = Math.abs (LeftEncoder.get()); 
      SlaveEncoder  = Math.abs (RightEncoder.get()); 
      Totalcounts += MasterEncoder;
      if (Totalcounts >= IntCounts) break;
      if (Totalcounts >= PartialCounts1) {
      	Adjust = REDUCE_SPEED_STEP_1; //* Reduce speed   *//
      }else if(Adjust < 1.0) {
      	Adjust *=1.05;     //* Ramp up the speed *//
      	if (Adjust > 1.0) Adjust = 1.0;
      }
      if (Totalcounts >= PartialCounts2) Adjust = REDUCE_SPEED_STEP_2; 
      Error = MasterEncoder - SlaveEncoder;
      // Adjust the Slave power by a factor of the error percentage - 
      SlavePower += (((double)Error/(double)MasterEncoder) * kP);
      if(SlavePower > 1.0) SlavePower = 1.0;
      if(SlavePower < (.85 * MasterPower)) SlavePower = MasterPower*.85;
      LeftFrontDriveValue  = FactorLF * MasterPower * Adjust;  
      LeftRearDriveValue   = FactorLR * MasterPower * Adjust;
      RightFrontDriveValue = FactorRF * SlavePower  * Adjust;
      RightRearDriveValue  = FactorRR * SlavePower  * Adjust;
      LeftFront.set(LeftFrontDriveValue);      //* Set Motors Power       *//
      RightFront.set(RightFrontDriveValue);
      LeftRear.set(LeftRearDriveValue);
      RightRear.set(RightRearDriveValue);
      RightEncoder.reset();               //* Clear the encoders          *//
      LeftEncoder.reset();                //* Clear the encoders          *//
      LoopCounter++;                      //* Increment the loop counter  *//
      Timer.delay(SAMPLERATE);            //* Set Sample Rate             *//
    }
    LeftFront.set(0.0);                   //* Stop Motors                 *//
    LeftRear.set(0.0);                    //* Stop Motors                 *//
    RightFront.set(0.0);                  //* Stop Motors                 *//
    RightRear.set(0.0);                   //* Stop Motors                 *//
 	  Timer.delay(0.2);                   //* Time for them to stop       *//
}
  
//*************************************************************************//
//*   Pivot Routine: Caller supplies Which (PIVOTL or PIVOTR) ,            *//
//*   /degrees +(Forward/Clockwise/Right) or -(Backward)                  *//
//*          and actual max power (0.2 to 1.0) to be used                 *//
//*               Pivot Side drive is considered master,                  *//
//*************************************************************************//
void RobotPivot(int which, double amount, double power)
{
	System.out.println(auto_state);
	  if(auto_state == false)
		  return;
    int    IntCounts, Totalcounts;   //* Encoder values for moves         *//
    int    Timeout, LoopCounter;     //* Calculate value for timeout      *//
    int    MasterEncoder;            //* Encoder values                   *// 
    int    PartialCounts1, PartialCounts2;//*Partial count to reduce speed*//
    double FactorLF   = -1.0;       //* Motor Direction,Left Side Reversed*//
    double FactorLR   = -1.0;       //* Motor Direction,Left Side Reversed*//
    double FactorRF   =  1.0;       //* Motor Direction                   *//
    double FactorRR   =  1.0;       //* Motor Direction                   *//
    double MasterPower, Counts;     //* Motor Power and Counts            *//
    double Adjust = 0.5;  //* Adjust factor for Soft start & slowing down *//
    boolean Swap_Encoders = false;; //* LPivot in Process Swap Encoders   *//
    boolean NoRamp = false;         //Prevents ramping down
    if (power < MINPOWER) return;
    MasterPower = power;            //* Use given power                   *// 
    if(MasterPower > MAXPOWER) MasterPower = MAXPOWER;//*                 *//
    switch (which) {
  	  case (PIVOTL):             //* Pivot on left side                 *//
   		  if (Math.abs(amount) < MINPIVOT)return; //* Must move enough  *//
  	      Swap_Encoders = true;
            if(amount < 0) {      //* Pivot to left                       *//
          	 Counts = -amount * PIVOT_COUNTS_PER_DEGREE_LEFT;
               FactorLF = 0.0;   //* Do Not drive  Left side              *//
               FactorLR = 0.0;   //* Right Side driven forward            *//
             }else{               //* Pivot to Right                      *//
          	  Counts = amount * PIVOT_COUNTS_PER_DEGREE_LEFT;
                FactorLR = 0.0;  //* Do not drive Left side               *//
                FactorLF = 0.0;
                FactorRF =-1.0;   //* Drive Right Side Backward           *//
                FactorRR =-1.0;
             }
  	      break;
  	  case (PIVOTR):             //* Pivot on right side                *//
   		  if (Math.abs(amount) < MINPIVOT)return; //* Must move enough  *//
            if(amount < 0) {      //* Pivot to left                       *//
          	 Counts = -amount * PIVOT_COUNTS_PER_DEGREE_RIGHT;
               FactorRR = 0.0;  //* Do not drive Right side               *//
               FactorRF = 0.0;
               FactorLF = 1.0;   //* Drive Left Side Backward             *//
               FactorLR = 1.0;
             }else{               //* Pivot to Right                      *//
          	  Counts = amount * PIVOT_COUNTS_PER_DEGREE_RIGHT;
                FactorRR = 0.0;  //* Do not drive Right side              *//
                FactorRF = 0.0;
                FactorLF =-1.0;   //* Drive Left Side Forward             *//
                FactorLR =-1.0;
             }
  	      break;
  	  case (DIAGF):             //* Diagonal forward to left or right  *//
   		  if (Math.abs(amount) < MINDIAG)return; //* Must move enough  *//
            NoRamp = true;        //* Prevent Ramping down
  	      Adjust = 0.7;
  	      if(amount < 0) {      //* Diagonal Forward to left           *//
        	     Swap_Encoders = true;
  	    	 Counts = -amount * DIAG_COUNTS_PER_INCH_LEFT;
               FactorLF =  0.10;   //* Stop LF & RR motors               *//
               FactorRR =  -0.10;
               FactorLR = -1.0;   //* Reverse LR & RF motors             *//
               FactorRF =  0.98;
             }else{               //* Diagonal forward to Right         *//
          	  Counts = amount * DIAG_COUNTS_PER_INCH_RIGHT;
                FactorLR = -0.1;  //* Stop LR & RF motors               *//
                FactorRF =  0.1;
                FactorLF = -0.98;   //* Forward LF & RR motors          *//
                FactorRR =  1.0;
             }
  	      break;
  	  case (DIAGB):             //* Diagonal Backward to left or right  *//
   		  if (Math.abs(amount) < MINDIAG)return; //* Must move enough   *//
            NoRamp = true;        //* Prevent Ramping down
  	      Adjust = 0.7;         //* Start out faster
  	      if(amount < 0) {      //* Diagonal backward to left           *//
                Counts = -amount * DIAG_COUNTS_PER_INCH_LEFT;
                FactorLR =  0.0;  //* Stop LR & RF motors                 *//
                FactorRF =  0.0;
                FactorLF =  1.0;   //* Reverse LF & RR motors             *//
                FactorRR = -1.0;
             }else{               //* Diagonal Backward to Right          *//
          	  Swap_Encoders = true;
          	  Counts = amount * DIAG_COUNTS_PER_INCH_RIGHT;
                FactorLF =  0.0;   //* Stop LF & RR motors                *//
                FactorRR =  0.0;
                FactorLR =  1.0;   //* Reverse LR & RF motors             *//
                FactorRF = -1.0;
             }
  	      break;
  	  default:  
  		  return;    // Unknown type of move
    }
    PartialCounts2 =(int)(Counts * MILESTONE_STEP_2); //* @ 70% slow it   *//
    PartialCounts1 =(int)(Counts * MILESTONE_STEP_1); //* @ 60% slow it   *//
    Timeout= (int) ((Counts/COUNTS_PER_SECOND) /(MasterPower*SAMPLERATE)); 
    Timeout= (int) ((Counts/COUNTS_PER_SECOND) /(MasterPower*SAMPLERATE)); 
    if (Timeout < MINLOOPCOUNT) Timeout = MINLOOPCOUNT; //* Minimum loops *//
    IntCounts = (int) Counts;        //* Integer value for the count      *//
    Totalcounts = 0; LoopCounter = 0; //* Clear starting values           *//
    if((Math.abs(amount) < 20.0)|| (power < 0.5))Adjust = 1.0;//*Too Small*//
    LeftFrontDriveValue = FactorLF * MasterPower * Adjust; //* Motor Power*//
    RightFrontDriveValue= FactorRF * MasterPower * Adjust; //* Motor Power*//
    LeftRearDriveValue  = FactorLR * MasterPower * Adjust; //* Motor Power*//
    RightRearDriveValue = FactorRR * MasterPower * Adjust; //* Motor Power*//
    LeftEncoder.reset();             //* Clear the encoders               *//
    RightEncoder.reset();            //* Clear the encoders               *//
    LeftFront.set(LeftFrontDriveValue);      //* Set Motors Power         *//
    RightFront.set(RightFrontDriveValue);    //* Set Motors Power         *//
    LeftRear.set(LeftRearDriveValue);        //* Set Motors Power         *//
    RightRear.set(RightRearDriveValue);      //* Set Motors Power         *//
    Timer.delay(0.3);                //* Time for motors to get started   *//
    while ((LoopCounter < Timeout) && auto_state == true){
//* This waits for motor  to reach desired position and then jumps out    *//
  	if(!Swap_Encoders) { //* Swap the encoders                      *//
         MasterEncoder = Math.abs (LeftEncoder.get()); 
   	}else {
         MasterEncoder = Math.abs (RightEncoder.get()); 
  	}
      Totalcounts += MasterEncoder;
      if (Totalcounts >= IntCounts) break;
      if ((Totalcounts >= PartialCounts1) && !NoRamp) {
      	Adjust = REDUCE_SPEED_STEP_1; //* Reduce speed   *//
      }else if(Adjust < 1.0) {
      	Adjust *=1.05;     //* Ramp up the speed *//
      	if (Adjust > 1.0) Adjust = 1.0;
      }
      if ((Totalcounts >= PartialCounts2) && !NoRamp)
      	Adjust = REDUCE_SPEED_STEP_2; 
      LeftFrontDriveValue  = FactorLF * MasterPower * Adjust;  
      LeftRearDriveValue   = FactorLR * MasterPower * Adjust;
      RightFrontDriveValue = FactorRF * MasterPower  * Adjust;
      RightRearDriveValue  = FactorRR * MasterPower  * Adjust;
      LeftFront.set(LeftFrontDriveValue);      //* Set Motors Power       *//
      RightFront.set(RightFrontDriveValue);
      LeftRear.set(LeftRearDriveValue);
      RightRear.set(RightRearDriveValue);
      RightEncoder.reset();               //* Clear the encoders          *//
      LeftEncoder.reset();                //* Clear the encoders          *//
      LoopCounter++;                      //* Increment the loop counter  *//
      Timer.delay(SAMPLERATE);            //* Set Sample Rate             *//
    }
    LeftFront.set(0.0);                   //* Stop Motors                 *//
    LeftRear.set(0.0);                    //* Stop Motors                 *//
    RightFront.set(0.0);                  //* Stop Motors                 *//
    RightRear.set(0.0);                   //* Stop Motors                 *//
 	Timer.delay(0.5);                     //* Time for them to stop       *//
}

	 public void ForwardDrive(double power, double time) {
       RightFront.set(power);
       LeftFront.set(-power);
       RightRear.set(power);
       LeftRear.set(-power); 
       Timer.delay(time);
	 }
	 
	 public void ForwardDist(double power, double dist) {
	       LeftEncoder.reset();
	       CountLeft = 0;
	       dist = dist * COUNTS_PER_INCH;
	       double starttime = Timer.getFPGATimestamp();
	       while(Math.abs(CountLeft) < dist || (Timer.getFPGATimestamp() - starttime) > 8) {
	       CountLeft = LeftEncoder.get();
		   RightFront.set(power);
	       LeftFront.set(-power + 0.15);
	       RightRear.set(power);
	       LeftRear.set(-power + 0.15); 
	       //Timer.delay(time);
	       }
	       Brake();
	 }
	 
	 public void TurnRightDist(double power, double dist) {
	       LeftEncoder.reset();
	       CountLeft = 0;
	       dist = dist * COUNTS_PER_INCH;
	       double starttime = Timer.getFPGATimestamp();
	       while(Math.abs(CountLeft) < dist && (Timer.getFPGATimestamp() - starttime) > 8) {
	    	   SmartDashboard.putNumber("Elapsed Time Right", Timer.getFPGATimestamp() - starttime);
	    	   CountLeft = LeftEncoder.get();
	    	   RightFront.set(-power);
	    	   LeftFront.set(-power + 0.15);
	    	   RightRear.set(-power);
	    	   LeftRear.set(-power + 0.15); 
	       //Timer.delay(time);
	       }
	       Brake();
	 }
	 
	 
	 public void TurnLeftDist(double power, double dist) {
	       RightEncoder.reset();
	       CountRight = 0;
	       dist = dist * COUNTS_PER_INCH;
	       double starttime = Timer.getFPGATimestamp();
	       while(Math.abs(CountRight) < dist && (Timer.getFPGATimestamp() - starttime) > 8) {	    	   
	    	   SmartDashboard.putNumber("Elapsed Time Left", Timer.getFPGATimestamp() - starttime);
	    	   CountRight = RightEncoder.get();
	    	   RightFront.set(power);
	    	   LeftFront.set(power - 0.15);
	    	   RightRear.set(power);
	    	   LeftRear.set(power - 0.15); 
	       //Timer.delay(time);
	       }
	       Brake();
	 }
	 
	 
	 public void ReverseDrive(double power, double time) {
	       RightFront.set(-power);
	       LeftFront.set(power - 0.15);
	       RightRear.set(-power);
	       LeftRear.set(power - 0.15); 
	       Timer.delay(time);
	 }
	 
	 public void LeftTurnDrive(double power, double time) {
	       RightFront.set(power);
	       LeftFront.set(power);
	       RightRear.set(power);
	       LeftRear.set(power); 
	       Timer.delay(time);
	 }
	 
	 public void RightTurnDrive(double power, double time) {
	       RightFront.set(-power);
	       LeftFront.set(-power);
	       RightRear.set(-power);
	       LeftRear.set(-power); 
	       Timer.delay(time);
	 }
	 
	 
	 public void StopDrive() {
       RightFront.set(0.0);
       LeftFront.set(0.0);
       RightRear.set(0.0);
       LeftRear.set(0.0); 
       Timer.delay(0.2);
	 }
	 
	 public void Brake() {
	       RightFront.set(-0.1);
	       LeftFront.set(0.1);
	       RightRear.set(-0.1);
	       LeftRear.set(0.1); 
	       Timer.delay(0.1);
	       StopDrive();
	 }
	 
	 public void setAuto(boolean auto) {
		 auto_state = auto;
	 }

}
