# FLASH Interface GUI - Summer Project

## FLASH Shutter Interface User Guide
### Introduction:
This application is designed for both control of and taking measurements from the FLASH shutter system at the UoB MC40 cyclotron.
The user interface displays a control panel on the left and a readout plot on the right. The readout plot displays the voltage measured by the Picoscope, which is proportional to the current on Faraday cup 1 of the cyclotron. This calibration is not expected to change, so it has been hard coded into the application. If it is necessary to change the calibration values, this can be done via the “setup.txt” file. It is possible to toggle between the raw Picoscope voltage and the equivalent current using the “Unit Toggle” button below the readout plot.
The beam which is incident upon Faraday cup 1 may go through any particular setup of collimation and attenuation for any experiment. As such, the relationship between the current on the Faraday cup and the current as measured by the Markus chamber at the end of the apparatus must be established at the start of a specific run before any measurements can be taken. This must be achieved by calibrating the system.

### Calibration:
To calibrate, first navigate to the ‘Calibration’ tab. Measurements of both Picoscope voltage and Markus chamber current must be entered into the table to be used in the calibration. Please note that the first row of the table is immediately below the header. If this is left blank, an error will be thrown. Please also remove any trailing blank rows before pressing the ‘Calibrate’ button.
Values entered into the table can be both saved as text files and recalled. This allows for a calibration done at the start of the day to be reused if the program is closed and reopened.
It is recommended that the calibration plot is displayed and inspected to ensure that data has been entered correctly and there have been no fitting issues. Once this has been completed, measurements can begin.

### Measurements:
Once calibrated, navigate back to the ‘FLASH’ tab, where a readout of picoscope voltage and the equivalent current and dose rates will be visible.
In the control panel on the left side of the application, the averaged values are displayed. These are averaged over the 1.5 seconds of data displayed on the readout plot to ensure a smooth readout. Below the readout plot, the instantaneous versions of the same parameters are visible.
A measurement may be performed in one of two ways:
1) Entering a required dose and dose rate and allowing the calibrated program to calculate the required cyclotron current and FLASH time.
2) Manually entering FLASH time and required cyclotron current.

Methods (1) and (2) can be toggled between via the ‘Manual Input’ checkbox.
Once a valid input (positive, real number) has been entered into both FLASH duration and required current, the “FLASH” button in the bottom left will change from grey (deactivated) to red (active).
A FLASH irradiation and measurement can now be performed. Toggling “Wait for required current” below the FLASH button will force the system to wait for the beam current to be stable on the Faraday cup before running the system and performing an irradiation.

IMPORTANT: Note that due to physical limitations of the FLASH shutter hardware, FLASH durations below Xms are not possible.
Once the FLASH button has been pressed, the FLASH shutter will open and close for the required amount of time. A photodiode readout from a laser perpendicular to the beam will produce a trace which is inverse to the irradiation of the target. The software will analyse this trace to determine the exact time for which the shutter was open. By toggling the “Show plots” checkbox, these traces and fits will be displayed for diagnosis. The pulse time will be displayed on the result readout in the bottom right of the screen. The total dose and dose rate are calculated from the current measurements before and after the irradiation and are also displayed.


## Future work:
- Include function that applies calibration of shutter opening time against requested time to send a signal to Arduino that *should* result in the requested time being FLASHed for
- Test with current read in from Cyclotron
- Test with photodiode read in from laser setup
- Edit Arduino script to raise and lower Faraday cup before flashing
