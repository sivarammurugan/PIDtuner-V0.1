# PIDtuner-V0.1

PlantPy PID tuner version 0.1

Features

Supports  first order  and ramp process models
Supports Proportional action on PV or  error  or a specific weighting between both.
Simulation of  open loop and closed loop responses.
Simulation setpoint and disturbance responses.
PID algorithm used is  non-interactive , derivative on PV. No derivative filtering in this version.
Anti reset windup implemented .MV minimum and maximum clamped at 0 and 100% respectively.

Usage

Click on the Process model header to expand . Choose the type of the process. Default is first order process . Check “Ramp” button if it is a ramp process. 
Enter process gain, time constant and delay . If the process is ramp, the time constant is not applicable and it is hidden. 
The units of parameters dont matter, but they have to be consistent.  if the process model parameters are entered in time units of minutes, the PID parameters should be interpreted in minutes . If entered in “Seconds” , PID parameters should be interpreted in time units of seconds. 
 Adjust the slider to increase or decrease the values of the parameters. To directly enter the values press  shift and click on the slider. 
Click on the disturbance model and Enter the disturbance model parameters if disturbance response is needed.
Enter the set point weighting .  Some DCS vendors  like Schneider Electric  Foxboro allow to enter the setpoint weighting  in the range of 0 to 1 (SPLLAG parameter) , for the proportional action. 
0  is the  proportional action on PV. Similar I-PD algorithm in Yokogawa DCS.  
1 - is the proportional action  on error.  similar to PI-D algorithm in Yokogawa DCS
Choose the number of steps to simulate.
Click on the “reset to lambda tuning”  to get tuning parameters based on entered process model and lambda tuning rule . You can not change the value of P, I, D on the sliders  , if “reset to lambda tuning” is selected. Uncheck it, to  fine-tune P,I and D.
Drag the sliders on P,I,D to change the values and simulate the closed loop response. 
To rescale the plots , double click on the plots. Right clicking on the plot brings up the context menu to adjust the scales etc . Zoom in and out is done using mouse wheel . To zoom in a specific section of a plot , draw a  rectangle with right mouse button. To reset the zoom , double click on the plot.



Comments ,  bugs and feature requests   are welcome , please send to  Sivaram.Murugan+tuner@gmail.com 
