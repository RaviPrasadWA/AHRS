# AHRS
AHRS ( Attitude Reference Heading System ) ONLY FOR HELICOPTER ( Flybarless 3-axis stablizer )

this ahrs is based of DCM Direction cosine matrix which is primarily used to determine the angle deviation in all X,Y,Z axis 
the <Attitude.h> takes the input from the PPM( pulse position modulation ) and mixes them with the PID 
( propotional integral derivative ) controller to give the corection values which are then mixed via CCPM 
(cyclic collective pitch mixing ) anf feed out to the servo.

Imagine a circle marked in compass points North is 0 degrees, East is 90 degrees, South 180 degrees and West 270 degrees. 
Now imagine cyc1 connected to North (0 degrees), cyc2 connected at ESE (120 degrees) and cyc3 connected at WSW (240 degrees).

If we push elevator forwards(positive), cyc1 will go down (negative) and cyc2 and cyc3 will go up (positive). So:

cyc1 = -cos(0)*ele
cyc2 = -cos(120)*ele
cyc3 = -cos(240)*ele


cos(0)=1
cos(120) = -0.5
cos(240) = -0.5

Now if we pull aileron left(positive), cyc1 will stay level, cyc2 will go up (positive), cyc3 will go down (negative). So:

cyc1 = sin(0)*ail
cyc2 = sin(120)*ail
cyc3 = sin(240)*ail

Again, recall from your high school trigonometry:

sin(0)=0
sin(120) = 0.866
sin(240) = -0.866

Now if we push collective up, cyc1, cyc2 and cyc3 will go up. So:

cyc1 = col
cyc2 = col
cyc3 = col

So if we add all the formulas together, we get:

cyc1 = (-cos(0)*ele) + (sin(0)*ail) + col
cyc2 = (-cos(120)*ele) + (sin(120)*ail) + col
cyc3 = (-cos(240)*ele) + (sin(240)*ail) + col

or when reduced for 120 CCPM:

cyc1 = col – ele
cyc2 = col + 0.5*ele + 0.866*ail
cyc3 = col + 0.5*ele – 0.866*ail


SPECIAL THANKS : To everybody around the world supporting open source knowledge/technology to everyone whose work enabled me to contruct this project
