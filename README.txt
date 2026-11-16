
                                    AHRS
-----------------------------------------------------------------
Summary:
	This sensor fusion project uses the accelerometer and magnetometer to correct the gyroscope's 
drift. Without correction, errors such as heating, bias instability, sensor noise, and environmental 
factors will add up to make the output unreliable.

Setup:
	SparkFun 9DoF IMU Breakout - ISM330DHCX, MMC5983MA (Qwiic) - sensor
	ELEGOO ESP32 Type-C - microcontroller
	Arduino IDE - IDE
	MATLAB - data analysis
	puTTy - data printout
	C++ - programming language

Wiring: 
            IMU     ||    ESP32
           -----------------------
            GND     ->    GND
            3V3     ->    3V3
            SDA     ->    GPIO 21
            SCL     ->    GPIO 22

Files:
	AHRS_master.ino - main file responsible for sensor software setup, data collection, and  
                          pipeline

	matrixMath.h/cpp - source file for matrix and vector math; also stores struct that initializes 
                           the gravity and magnetic reference, calibration/alignment matrices and 
                           vectors 

	moreMath.h/cpp - source file for miscellaneous math functions (e.g: normalization, conversion, 
                         vector to plane projection, etc)
	sensorFusion.h/cpp - source file for gyro integration with accel and mag correction/PID 
                             controller

	eulerQuaternion.h/cpp - source file for extracting euler angles and quaternion from DCM; also
                                includes quaternion math functions 

	gyros_nullShift_scaleFactor.m

	magTumble.m

	accel_calibration.m

	DCMAlignAccel.m

	WhabaSVD.m

Pipeline:
At Setup:
	collect initial accel and mag data (sensitivity conversion)
                                ⇓
	calibrate accel and mag data
                                ⇓
        align the calibrated accel and mag to the runtime fusion frame
                                ⇓
	normalize the aligned accel and mag
                                ⇓
        generate the initial DCM from TRIAD method using the normalized accel and mag
                                ⇓
                            
At Loop:
	
	continuously collect accel, mag, and gyro data (sensitivity conversion)
                               ⇓
        calibrate accel, mag, and gyro data
                               ⇓
	align the calibrated accel, mag, and gyro to the runtime fusion frame
                               ⇓
	normalize the aligned accel and mag
			       ⇓
	integrate gyro with accel and mag correction and initial DCM; generates a new DCM
                               ⇓
	transpose DCM
                               ⇓
	extract the quaternion and euler angles from DCM
			       ⇓
	print out or apply outputs 

                             
Calibration Method: 



Rotations: 
	roll = rotation about X
	pitch = rotation about Y
	yaw = rotation about Z
AHRS Body Frame Map:

	Board Poses     || accel x || accel y || accel z
        -------------------------------------------------
	flat	          0.02826   -0.01513    0.99949
	upside down      -0.05538   -0.02704   -0.99810
	*nose up          0.00557   -0.99998   -0.00200
	*nose down        0.02706    0.99939   -0.02203
	*right side down  0.99952   -0.01366   -0.02785
	*left side down  -0.99689    0.01362    0.07759
		*~ assumed nose is direction when yaw ~0 deg when pointed north

	
	||       AHRS Body FRAME       || 
        ---------------------------------
	||+X = right side of the board ||
	||-X = left side of the board  ||
	||+Y = tail side of the board  ||
	||-Y = nose side of the board  ||
	||+Z = up                      ||
	||-Z = down                    ||
     



AHRS Performance Statistics
16 Sept 2026
---------------------------- 
		(YAW Verification)
1) Power-Up Repeatability
*Turn on/plugged in board, wait 10 secs, and note start up yaw. 
*Turn off/unplug board, wait 10 secs, and repeat first step.
*Repeat until enough data for mean, standard deviation, and maximum deviation analysis.
	Run || Startup Yaw
	------------------
	1	6.89
	2	6.78
	3	6.49
	4	6.75		
	5	6.59
	6	6.60	
	7	7.08
	8	6.96
	9	7.14
	10	7.01
Mean: 6.8290
Standard Deviation: 0.2227
Maximum Deviation from Mean: 0.3390

2) Cardinal Heading Test
*Turn on/plug in board and print out yaw and heading error.
*Use a phone compass to mark North, East, South, West.
*Point board to the cardinal headings and verify if they are 90 deg part. 
**Note that positive and negative outputs range from 0 to (+- 180)
*Verify heading error is < 1 or 2
Cardinal ||Expected || Reported || Heading Error
-----------------------------------------------
     N       0		2.06	     0.47
     E	     90	       -88.80	     0.04
     S	    180	       -175.85       2.07
     W	    270		95.18        0.17
     N	      0         3.09         0.58

3) 360 Closure Test
*Turn on/plug in board and print out yaw
*Take note of initial yaw and rotate board 360 deg
*Verify that the difference between final and initial yaw is minimal
	Trial || Starting Yaw || Ending Yaw || Closure Error
	----------------------------------------------------
	1	     -77.65       -75.11       2.54 
	2            87.35         85.65       1.70     
	3	     149.56	   148.76      0.20

4) Stationary Drift (5 mins)
*Turn on/plug in board and print out yaw
*Take note of initial yaw and keep board still for 5 minutes
*Verify that the difference between final and initial yaw is minimal
	Trial || Starting Yaw || Ending Yaw || Drift
        ---------------------------------------------	
	1           -12.69        -11.80        0.89
	2            161.55	  162.93        1.38
	3           -137.29       136.71        0.58     


AHRS Performance Summary

Power-Up Repeatability
Mean Startup Yaw: 6.83°
Std Dev: 0.22°
Max Deviation: 0.34°

Cardinal Heading Accuracy
Average Error: 0.67°
Maximum Error: 2.07°

360° Closure Test
Average Closure Error: 1.48°
Maximum Closure Error: 2.54°

Stationary Drift (5 min)
Average Drift: 0.95°
Maximum Drift: 1.38°

Conclusion:
TRIAD initialization, heading correction,
and gyro bias compensation are operating
correctly. Measured performance indicates
sub-degree/minute stationary stability and
~2° heading accuracy.

AHRS Performance Statistics
17 Sept 2026
-------------------------
	(Pitch and Roll Verification)
1) Six-Face Static Attitude Test & Intermediate Angle Test

	         Pose        || Roll    ||    Pitch    || Yaw
	--------------------------------------------------------
	Flat                   2.13          -2.67      -37.61                  	
	Upside Down            176.75         2.64      -41.70           
	Right Side Down       -90.98          0.10      -144.91                             
	Left Side Down         90.37         -0.71      -5.58
	Nose Up                64.07         86.63      -38.84     
	Nose Down             -168.46       -86.07      -61.73
          *note that 90 deg roll and pitch will reach singularity/gimbal lock, 
	   and therefore not reliable
	         Pose        || Roll    ||    Pitch    || Yaw
	--------------------------------------------------------
	Flat                    1.45          -2.67         51.63
	Upside Down             176.54         2.74        -122.23  
	Roll 30 deg             33.27         -3.00         11.89        
	Roll 60 deg             60.68         -1.95         11.50
	Roll -30 deg           -31.45         -1.22        -93.31
	Roll -60 deg           -59.54         -1.07        -114.74
	Pitch 30 deg            2.94           28.65       -42.91
	Pitch 60 deg            2.89           57.67       -60.41
	Pitch -30 deg          -0.87          -33.55        172.67
	Pitch -60 deg           2.38           -63.49       145.34


2) Power-Up Repeatability
	Flat Trial    || Roll    ||    Pitch   
	---------------------------------------
	1		0.18 	     -2.54
	2		0.03 	     -2.50
	3		0.05         -2.51
	4 		0.04         -2.50
	5       	0.26         -2.57
	6		-0.07        -2.49
	7		0.24         -2.56 
	8		-0.02        -2.51
	9		0.26         -2.56
	10		0.03         -2.51
Roll:
	Mean: 0.1000
	Standard Deviation: 0.1231
	Maximum Deviation from Mean: 0.1700
Pitch:
	Mean: -2.5250
	Standard Deviation: 0.0295
	Maximum Deviation from Mean: 0.0450

	Upside Down Trial    || Roll    ||    Pitch   
	---------------------------------------
	1			178.64 	      2.35
	2			178.84        2.41
	3			178.79        2.40
	4			178.64        2.37
	5			178.78        2.38
	6			178.74        2.37
	7			178.72        2.36 
	8			178.52        2.31 
	9			178.55        2.32	
	10			178.76        2.36 
Roll:
	Mean: 178.6980
	Standard Deviation: 0.1063
	Maximum Deviation from Mean: 0.1780
Pitch:
	Mean: 2.3630
	Standard Deviation: 0.0313
	Maximum Deviation from Mean: 0.0530

	Roll 30 deg Trial    || Roll    ||    Pitch   
	---------------------------------------
	1			31.66      -3.54
	2			32.14      -3.60
	3			31.55      -3.53
	4			31.79      -3.58 
	5			31.62      -3.56 
	6			31.83      -3.60
	7			31.94      -3.61
	8			31.47      -3.54
	9			31.86      -3.60
	10			31.51      -3.55
Roll:
	Mean: 31.7370
	Standard Deviation: 0.2128
	Maximum Deviation from Mean: 0.4030
Pitch:
	Mean: -3.5710
	Standard Deviation: 0.0303
	Maximum Deviation from Mean: 0.0410


	Roll -30 deg Trial    || Roll    ||    Pitch   
	---------------------------------------
	1			-30.78       -2.74
	2			-30.86       -2.71
	3			-30.85       -2.70
	4			-30.86       -2.67
	5			-30.87       -2.67
	6			-30.86       -2.68 
	7			-30.90       -2.65 
	8			-30.85       -2.68
	9			-30.81       -2.76
	10			-30.81       -2.70
Roll:
	Mean: -30.8450
	Standard Deviation: 0.0350
	Maximum Deviation from Mean: 0.0650
Pitch:
	Mean: -2.6960
	Standard Deviation: 0.0337
	Maximum Deviation from Mean: 0.0640

	
	 Pitch 30 deg Trial  || Roll    ||    Pitch   
	---------------------------------------
	1			-0.26       29.03
	2			-0.45       28.75
	3			-0.64       28.66
	4			-0.46       28.73
	5			-0.32       28.75
	6			-0.34       28.79 
	7			-0.55       28.70
	8			-0.43       28.74
	9			-0.38       28.75
	10			-0.25       28.79
Roll:
	Mean: -0.4080
	Standard Deviation: 0.1243
	Maximum Deviation from Mean: 0.2320
Pitch:
	Mean: 28.7690
	Standard Deviation: 0.0995
	Maximum Deviation from Mean: 0.2610

	Pitch -30 deg Trial  || Roll    ||    Pitch   
	---------------------------------------
	1			1.32        -33.47 
	2			1.52        -33.61
	3			1.52        -33.61 
	4			1.31        -33.47
	5			1.61        -33.69
	6			1.37        -33.51
	7			1.22        -33.41
	8			1.24        -33.43
	9			1.33        -33.49
	10			1.25        -33.43

Roll:
	Mean: 1.3690
	Standard Deviation: 0.1350
	Maximum Deviation from Mean: 0.2410
Pitch:
	Mean: -33.5120
	Standard Deviation: 0.0935
	Maximum Deviation from Mean: 0.1780


3) Five-minute roll/pitch Drift
	Trial || Starting Roll || Ending Roll || Drift Roll ||Starting Pitch || Ending Pitch || Drift Pitch
        ----------------------------------------------------------------------------------------------------	
	1        0.23                0.28           0.05         -1.57             -1.55           0.02         
	2        0.18                0.00           0.18         -1.39             -1.46           0.07
	3       -0.20               -0.08           0.12         -1.36             -1.45           0.09


4) Return to Level
	Trial || Starting Roll || Ending Roll || Closure Error
	----------------------------------------------------
	1	  0.65              0.56             0.09
	2         0.50              0.21             0.29
	3         0.09             -0.04             0.13
	4	  0.44             -0.78             1.22
	5         0.52             -0.63             1.15
	6         -0.67            -0.49             0.18      
	7         0.25              0.11             0.14
	8         0.40              0.34             0.06
	9         0.28              0.33             0.05
	10        0.17              0.36             0.19

	Trial || Starting Pitch || Ending Pitch || Closure Error
	----------------------------------------------------
	1	  -1.12            -1.14             0.02
	2         -1.88            -1.86             0.02
	3         -1.47            -1.48             0.01
	4         -1.91            -1.89             0.02
	5         -1.61            -1.64             0.03
	6         -1.29            -1.96             0.67
	7         -2.64            -2.02             0.62
	8         -1.73            -1.48             0.25
	9         -2.30            -2.43             0.13
	10        -2.44            -2.13             0.31


5) Dynamic Recovery 
*Note initial roll and pitch flat on table
*Shake board for 10 sec
*Place board back flat on table
		
Observation: Roll and pitch values jump as I tumbled the board for 10 seconds. When I set the board 
down and waited another 10 seconds, the values consistently settled between -1.5 to 1 deg.  		
      

Roll & Pitch Performance Summary

Roll
-----
Static Accuracy:
Average Error = 1.47°

Power-Up Repeatability:
Std Dev = 0.21°

5-Minute Drift:
Average = 0.12°

Return-To-Level:
Maximum Error = 1.22°


Pitch
------
Static Accuracy:
Average Error = 2.68°

Power-Up Repeatability:
Std Dev = 0.10°

5-Minute Drift:
Average = 0.06°

Return-To-Level:
Maximum Error = 0.67°
	   



