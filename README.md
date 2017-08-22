# Oil-Burner V0.6.3
By Max Sauer
Code in Work

	This is the code for the WVO Controller development for my pickup. It is intended
	to handle the switching of fuel control valves and display important information on
	the touch screen display.

	The Code is written for use with an Arduino Mega, and may potentially be rewritten for
	an arduino micro.

	A Seeedstudio 2.8" TFT Touch Shield is used. A touch screen breakout may be used in the future.

	The circuit:
	*INPUTS:
		A10 - Photodiode for Auto Backlight Adjustment
		A11 - WVO Filter Head TEMP
    		A14 - Fuel Pressure Sender
    		A15 - Fuel Level Sender
    
	Future Implementations:
		Analog - Engine Coolant Temperature


	*OUTPUTS:
		22 - Piezo Buzzer Pin
		24 - Return Valve MOSFET Gate
		26 - Supply Valve/Fuel Pump MOSFET Gate

	Touchscreen Features:
		grid coordinates start: (0, 0) end: (239, 319)




	Changelog:
 	 0.1.0   -   2/1/2015    -   Initial Prototype with TFT
  
  	0.2.0   -   2/24/2015   -   Features added: Buzzer, Temp Sensor, Mosfets for Switching Valves
  
 	 0.3.0   -   3/15/2015   -   Temperature Set Point and Brightness Menu Options Added, creating Settings Window to Hold all settings options
  
  	0.4.0   -   4/20/2015   -   EGT Gage Added
  
 	 0.5.0   -   unknown     -   Eeprom function added to save settings
  
  	0.5.1   -   8/25/2015   -   Fixed Bug allowing access to the EGT screen while the controller was in Purge Mode. 
  
  	0.5.2   -   8/26/2015   -   Brightness Scaling Equation Improved. Brightness button now has a smoother transition from dim to bright. 
  
  	0.6.0   -   5/17/2016   -   Auto Brightness Equation Added. Photodiode added to hardware to sense Ambient light for Auto Brightness Equation. Changed Lettering to White. 
 
  	0.6.1   -   6/3/2016    -   Added Fuel Gage functionality.
  
  	0.6.2   -   8/1/2016    -   Bug Fixes which causes fuel gage lines not to display when reloading homescreen
                          -   Added a Cancel Button to the Purging Page, which allows the purge sequence to be cancelled in the event of an accidental Purge
			  
  	0.6.3   -   8/21/2017   -   Added Fuel Pressure to display. This utilizes a 3 wire fuel pressure sender. More info at https://www.amazon.com/gp/product/B00RCPDE40/ref=oh_aui_search_detailpage?ie=UTF8&psc=1
                          -   fixed bug causing cancel button not to revert back to oil
                          -   added alarm for fuel pressure when on oil if pressure falls below 6 psi, also added color to fuel pressure indicator on screen 
