'''
Creation date: 11/8/22
Author: Max Conroy

Descrition:

    A QT interface that allows the user to complete FLASH experiments using 
    the proton beam at the Birmingham MC40 cyclotron.

    When connected to both a current input from Faraday cup 1 of the cyclotron 
    and the output of the timing photodiode, the following features are
    available:

    - Live monitoring of beam current
    - Calibration of beam current to dose rate at target
    - Calculation of required inputs from desired output dose + dose rate
    - Control of FLASH shutter to administer a pulse of radiation of a user
      specified length
    - Automatic triggering to record & analyse a photodiode pulse for accurate
      timing
    - Output of pulse duration as well as infered measurement of average dose 
      rate for duration of pulse.
'''

# Imports

# GUI
import sys

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QTableWidgetItem
)
from PyQt5 import QtTest, QtGui
from PyQt5.QtCore import Qt, QTimer

import pyqtgraph as pg

import matplotlib
matplotlib.use('Qt5Agg')

# Python files of GUI design
from flash_interface_gui4 import Ui_MainWindow
from help_popup import Help_MainWindow

# Processing
from threading import Thread
import serial
import numpy as np
import time
from datetime import datetime
from scipy.optimize import curve_fit
import matplotlib.pyplot as plt

# Python file for pulse analysis
import flashpulseanalysis3 as fp

# Picoscope packages
from ctypes import POINTER, c_int16, c_uint32

from picosdk.ps2000 import ps2000 as ps
from picosdk.functions import assert_pico2000_ok
from picosdk.PicoDeviceEnums import picoEnum
from picosdk.ctypes_wrapper import C_CALLBACK_FUNCTION_FACTORY
from picosdk.errors import (PicoSDKCtypesError, CannotFindPicoSDKError,
    CannotOpenPicoSDKError)

# Main window of app
class Window(QMainWindow):

    def __init__(self, parent=None):

        super().__init__(parent)

        # Load setup data from text file
        self.setupFile()

        # Number of data points to be stored and plotted 
        self.points = int(self.setup_params["Sample_points"])               
        # Trigger threshold in mV
        self.trigger_thresh = int(self.setup_params["Trigger_threshold"])   
        # Boolean values that control trigger
        self.triggered = False
        self.triggered_2 = False
        self.arm_trigger = False
        # Boolean value to pause / run updating of plot
        self.pause_plot = False

        # Set unit conversion parameters, convert to list of floats
        FDC_str = self.setup_params['FC_calibration'].split(', ')   
        self.FDC_conversion = [float(x) for x in FDC_str]           
        self.MK_conversion = 1, 0   # Will change when user calibrates

        # Boolean value of calibration
        self.calibrated = False

        # Boolean value of analysis
        self.analysed = False

        # Create an instance of the GUI
        self.ui = Ui_MainWindow()
        # Run the .setupUi() method to show the GUI
        self.ui.setupUi(self)
        
        # Disable FLASH button by default
        self.ui.flashButton.setEnabled(False)
        # Check if FLASH button can be enabled upon text change
        self.ui.flashDurationEntry.textChanged.connect(self.enableFlash)
        self.ui.requiredCurrentEntry.textChanged.connect(self.enableFlash)
        
        # Define button functionality
        self.ui.flashButton.clicked.connect(self.Flash)
        self.ui.calculateButton.clicked.connect(self.calculateFlashParams)
        self.ui.calibrateButton.clicked.connect(self.calibration)
        self.ui.removeRowButton.clicked.connect(self.tab_remove)
        self.ui.addRowButton.clicked.connect(self.tab_add)
        self.ui.saveCalibrationButton.clicked.connect(self.calib_save)
        self.ui.loadCalibrationButton.clicked.connect(self.calib_load)
        self.ui.unitToggle.clicked.connect(self.changePlotUnits)

        # Hide all error messages
        self.ui.flashInputErrorLabel.setText("")
        self.ui.calibrationErrorLabel.setText("")
        self.ui.FlashButtonError.setText("")

        # Set progress bar to default off
        self.ui.progressBar.hide()

        # Set FLASH result tab to default off (1 = index of tab)
        self.ui.plotTabs.setTabVisible(1, False)

        # Configure menu actions
        self.ui.openGuide.triggered.connect(self.showHelp)
        self.ui.actionExit.triggered.connect(self.close)

        # Set FLASH duration & current entries to be uneditable unless toggled
        self.ui.requiredCurrentEntry.setDisabled(True)
        self.ui.flashDurationEntry.setDisabled(True)
        self.ui.requiredVoltageEntry.setDisabled(True)

        # Create and update serial plots
        styles = {'color':'k', 'font-size':'10pt'}  # CSS styles  
        # Setup and style graph for channel A
        self.ui.graphWidget.setBackground('w')     
        y_max = int(self.setup_params["Pico_max_voltage"])                                
        self.ui.graphWidget.setYRange(0, y_max, padding=0)
        title = "Faraday Cup Current"                       
        self.ui.graphWidget.setTitle(title, color="k", size="14pt") 
                                     
        self.ui.graphWidget.setLabel('left', 'Voltage (mV)', **styles)
        self.ui.graphWidget.setLabel('bottom', 'Time (ms)', **styles)

        # Setup and style graph for channel B
        self.ui.graphWidget_2.setBackground('w')
        self.ui.graphWidget_2.setYRange(-100, 2500, padding=0)
        title = "FLASH Pulse Measurement"
        self.ui.graphWidget_2.setTitle(title, color="k", size="14pt")
        self.ui.graphWidget_2.setLabel('left', 'Voltage (mV)', **styles)
        self.ui.graphWidget_2.setLabel('bottom', 'Time (ms)', **styles)

        # Initial data to plot
        self.x = np.arange(0,self.points,1)
        self.y = []

        # Define line styles to be plotted for channels A and B (RGB colour)
        red_pen = pg.mkPen(color=(255, 0, 0), style=Qt.SolidLine)
        blue_pen = pg.mkPen(color=(0, 0, 255), style=Qt.SolidLine)

        # Define line and scatter plots, with separate pens
        self.data_line =  self.ui.graphWidget.plot(self.x, self.y, pen=red_pen)
        self.scatter = pg.ScatterPlotItem(size=10, pen=blue_pen, symbol='x')
        self.ui.graphWidget_2.addItem(self.scatter)
        self.fit_line = self.ui.graphWidget_2.plot([0], [0], pen=red_pen)

        # Attempt to connect to Arduino
        try:
            # Connect to serial port
            self.arduino = serial.Serial(self.setup_params["Arduino_port"],
                9600, timeout=1)
            self.arduino.flush()
            print('Connecting to serial...')
        
        except serial.serialutil.SerialException as E:
            print("Could not connect to Arduino.")


        # Attempt to connect to Picoscope
        try:
            # Test connection to Picoscope by opening and closing connection
            picoTest = ps.ps2000_open_unit()
            assert_pico2000_ok(picoTest)
            ps.ps2000_close_unit(picoTest)
        
            # Initialise thread to handle capture of data from oscilloscope
            picoThread = Thread(target = self.picoGet)
            picoThread.daemon = True
            picoThread.start()
            # Run loop every X ms to update plot with most recent data
            # NOTE: Cannot trigger on pulses shorter than X
            self.plot_timer = QTimer()                               
            self.plot_timer.setInterval(20)                          
            self.plot_timer.timeout.connect(self.update_plot_data)   
            self.plot_timer.start()


        except PicoSDKCtypesError:
            print("Could not connect to Picoscope.")

        except CannotFindPicoSDKError as e:
            print(e)

        except CannotOpenPicoSDKError as e:
            print(e)

                                          
        

    def setupFile(self):
        """
        Gets setup data from 'setup.txt' and reads it into dictionary.
        Uses LHS of entry as key and RHS as value. Splits on ": " in text file.
        """
        self.setup_params = {}

        f = open("setup.txt", "r")
        for line in f.readlines():
            insert = (line.split(': '))
            self.setup_params.update({insert[0] : insert[1].strip()})
        
        f.close()
        
    
    def enableFlash(self):
        """
        Method to test whether FLASH button can be enabled or disabled.

        FLASH button should only be enabled if there is a valid entry in the 
        FLASH duration text box. This is because the Arduino script must only 
        recieve integer/float values.

        Uses try/except to test for numerical input by attempting to convert 
        inputs to floats. If successful, enables FLASH button and changes 
        colour from grey to red. If unsuccessful, triggers exception without 
        throwing error / crashing program, sets background colour to grey and 
        disables button. This allows for button to be changed from red to grey
        if an invalid entry is provided.

        Function should be called every time text is updated in text entry 
        boxes, to give real time feedback as to whether input is valid.

        """        

        try:
            # Check if inputs are numerical values by converting to floats
            # Will fail if any non numerical (0-9) characters are input
            float(self.ui.flashDurationEntry.text())
            float(self.ui.requiredCurrentEntry.text())
            #float(self.ui.requiredVoltageEntry.text())
            # Set button enabled and set style to enabled
            self.ui.flashButton.setStyleSheet("background-color:red;"
                                                            "color: white;")
            self.ui.flashButton.setEnabled(True)

        except:
            # Set button + style to disabled
            self.ui.flashButton.setStyleSheet("background-color:gray;" 
                                                "color: white;" "border: none")
            self.ui.flashButton.setEnabled(False)
        

    def picoGet(self):
        """
        Gets data from Picoscope 2000 series oscilloscope.

        IMPORTANT: Must not run in main thread!

        Connects to picoscope device and sets up data collection, as per 
        PicoScope 2000 Series Programmer's Guide:
        https://www.picotech.com/download/manuals/picoscope-2000-series-programmers-guide.pdf. 

        Once connection has been made, gets data from buffer every microsecond. 
        Pause is required to prevent impacting performance too much.
        """
        
        # C parameters that are to be used in callback function, as required by
        # Picoscope API - see Programmer's Guide 5.34 for details
        CALLBACK = C_CALLBACK_FUNCTION_FACTORY(
            None,                       
            POINTER(POINTER(c_int16)),  # overviewBuffers pointer
            c_int16,                    # overflow
            c_uint32,                   # triggeredAt
            c_int16,                    # triggered
            c_int16,                    # auto_stop
            c_uint32                    # nValues
            )

        # Numpy array to hold data from oscilloscope in circular buffer
        self.adc_arr = np.zeros((self.points, 2))

        # Start and end pointers to track data placement in circular buffer
        self.start_pointer = 0
        self.end_pointer = 0


        def get_overview_buffers(buffers, _overflow, _triggered_at, _triggered,
                                    _auto_stop, n_values):
            """
            Gets data from device driver buffer & copies into circular buffer.

            Circular buffer enables constant aquisition without losing historic 
            data, up to a defined limit. Pointers track the start and end point
            of where to input data, filling array and then overwriting oldest 
            data when array is full.

            Args: See Programmer's Guide section 5.34
            """            

            # Set end pointer to n_values greater than start pointer, where 
            # n_values is number of values to add. Can go above max index of 
            # adc_arr
            self.end_pointer = (self.start_pointer + n_values)

            # Check if end pointer is within limits of adc_arr
            if self.end_pointer < self.points:
                # If it is, all data can be inserted into array
                # Index 0 is chA_max, index 2 is chB_max
                self.adc_arr[self.start_pointer:self.end_pointer,0] = \
                                                        buffers[0][0:n_values]
                self.adc_arr[self.start_pointer:self.end_pointer,1] = \
                                                        buffers[2][0:n_values]

            # Check if end pointer is outside of limits of adc_arr
            elif self.end_pointer >= self.points:
                # If it is, define mid_pointer to be equal to number of indices
                # between start pointer and end value of array
                mid_pointer = self.points - self.start_pointer
                # Store first N values from buffer into array, where N is max 
                # number of values that can fit from start pointer to max
                self.adc_arr[self.start_pointer:self.points,0] = \
                                                    buffers[0][0:mid_pointer]
                self.adc_arr[self.start_pointer:self.points,1] = \
                                                    buffers[2][0:mid_pointer]
                # Calculate new end pointer as remainder when dividing by max 
                # index (e.g. 1500 % 1000 = 500)
                self.end_pointer = (self.end_pointer) % self.points
                # Set start pointer to 0, first index of array
                self.start_pointer = 0
                # Store remaining values from buffer into start of adc_arr
                self.adc_arr[self.start_pointer:self.end_pointer,0] = \
                                            buffers[0][mid_pointer:n_values]
                self.adc_arr[self.start_pointer:self.end_pointer,1] = \
                                            buffers[2][mid_pointer:n_values]
        
            # Reset start pointer for next data aquisition
            self.start_pointer = (self.end_pointer)

            # Pointer to trigger value is 20% from start of array
            trig_pointer = ((self.start_pointer + int(self.points/5))
                                                                % self.points)

            # Check to see if value at trigger pointer is over trigger 
            # threshold, set boolean triggered attribute accordingly
            channel_index = picoEnum.PICO_CHANNEL[
                                    (self.setup_params['Trigger_channel'])]
            if self.adc_arr[trig_pointer, channel_index] > self.trigger_thresh:
                self.triggered = True
            else:
                self.triggered = False


        # Define callback as Python object
        callback = CALLBACK(get_overview_buffers)

        # Code to interface with picoscope
        with ps.open_unit() as device:
            print('Connecting to Picoscope: {}'.format(device.info))

            # Sets up channel A for data aquisition
            setup_chA = ps.ps2000_set_channel(
                device.handle,                              # Device name 
                picoEnum.PICO_CHANNEL['PICO_CHANNEL_A'],    # Channel to access
                True,                                       # Enabled/disabled
                picoEnum.PICO_COUPLING['PICO_DC'],          # Coupling type
                ps.PS2000_VOLTAGE_RANGE['PS2000_5V'],       # Voltage range
            )
            # Check for errors
            assert_pico2000_ok(setup_chA)

            # Sets up channel B for data aquisition
            setup_chB = ps.ps2000_set_channel(
                device.handle,                              # Device name 
                picoEnum.PICO_CHANNEL['PICO_CHANNEL_B'],    # Channel to access
                True,                                       # Enabled/disabled
                picoEnum.PICO_COUPLING['PICO_DC'],          # Coupling type 
                ps.PS2000_VOLTAGE_RANGE['PS2000_5V'],       # Voltage range
            )
            # Check for errors
            assert_pico2000_ok(setup_chB)

            # Sets up streaming data aquisition
            setup_streaming = ps.ps2000_run_streaming_ns(
                device.handle,                              # Device name 
                int(self.setup_params["Sample_interval"]),  # Sample interval
                picoEnum.PICO_TIME_UNITS[                   #   (in time units)
                    (self.setup_params['Time_units'])],     # Time units
                100_000,                                    # Max samples
                False,                                      # Autostop
                1,                                          # Aggregation no.
                15_000                                      # Driver buffer
            )
            # Check for errors
            assert_pico2000_ok(setup_streaming)
            print("Connected")
            
            # Loop that gets data from driver buffer by running callback fn
            while True: 
                # Run callback function to get data from oscilloscope
                ps.ps2000_get_streaming_last_values(
                    device.handle,  # Device name to access
                    callback        # Callback function to run
                )
                # Pause for 1 ms to prevent lag in GUI
                QtTest.QTest.qWait(1)
                    

    def update_plot_data(self):
        """
        Method that runs on timer to update plot with circular buffer data.

        Gets data from circular buffer, using start pointer to ensure most 
        recent data is plotted first.

        """

        # Function that converts raw ADC values to voltage 
        def adc_to_mv(values, range, bitness = 16):
            v_ranges = [10, 20, 50, 100, 200, 500, 1_000, 
                            2_000, 5_000, 10_000, 20_000]
            mV = values * v_ranges[range] / (2**(bitness-1) - 1)
            return mV

        # Get two sets of data: from start pointer to end of array...
        data_1 = self.adc_arr[self.start_pointer:self.points,:]
        #...and from start of array to end pointer
        data_2 = self.adc_arr[0:self.start_pointer,:]

        # Combine data into single chronological array
        data_tot = np.concatenate((data_1, data_2))

        # Convert to voltage, as defined by range of oscilloscope
        voltage = adc_to_mv(data_tot, ps.PS2000_VOLTAGE_RANGE['PS2000_5V'])
        # Factor to convert points to readable time (/1000 for easier reading)
        t_conversion = int(self.setup_params["Sample_interval"]) / 1000
        # Create array of times, same size as voltages
        time = np.arange(0,self.points,1) * t_conversion

        # Combine data
        self.FCD_data = np.vstack((time, voltage[:,0])).T
        self.laser_data = np.vstack((time, voltage[:,1])).T

        # Plot if not paused
        if self.pause_plot == False:
            
            # Convert data using pre-defined FDC calibration if selected
            if self.ui.unitToggle.isChecked() == True:
            
                # Channel A plot is converted by FDC_conversion
                converted = (self.FCD_data[:,1] * (self.FDC_conversion[0]) 
                                                    + (self.FDC_conversion[1]))

                self.data_line.setData(self.FCD_data[:,0], converted)
            
            else:

                # Channel A plot is left unconverted
                self.data_line.setData(self.FCD_data[:,0], self.FCD_data[:,1])

            # Update text labels with average over entire plot
            avg_voltage_A = np.round(np.mean(voltage[:,0]), 0) 
            self.ui.avgPicoVoltage.setText(
                f'Averaged Voltage: {avg_voltage_A} mV')
            
            # Update instantaneous readout labels
            self.ui.instVoltageLabel.setText(
                f"Instantaneous Voltage: {np.round(voltage[-1,0], 3)} mV"
                )
            inst_current = (voltage[-1,0] * (self.FDC_conversion[0]) 
                                                    + (self.FDC_conversion[1]))
            self.ui.equivCurrent.setText(
                f"Equivalent Faraday Cup Current: {np.round(inst_current, 3)} nA"
                )

            # Update labels that depend on calibration, only if calibrated
            if self.calibrated == True:

                # Calculate MC current and dose rate from calibration
                MC_current = ((avg_voltage_A - self.MK_conversion[1]) 
                                                    / self.MK_conversion[0])
                dose_rate = (MC_current
                    * float(self.setup_params["Mk_chamb_dose_coefficient"]))

                # Round for output
                MC_current = np.round(MC_current, 3)
                dose_rate = np.round(dose_rate, 3)

                # Update labels
                label_text = f'Averaged Markus Chamber Current: {MC_current} nA'
                self.ui.avgCurrentLabel.setText(label_text)
                label_text = f'Averaged Dose Rate: {dose_rate} Gy/s'
                self.ui.avgDoseRate.setText(label_text)

                # Update instantaneous label
                inst_MC_current = ((voltage[-1,0] - self.MK_conversion[1]) 
                                                    / self.MK_conversion[0])
                inst_dose_rate = (inst_MC_current
                    * float(self.setup_params["Mk_chamb_dose_coefficient"]))

                self.ui.equivDoseRate.setText(
                f"Equivalent Faraday Cup Current: {np.round(inst_dose_rate, 3)} nA"
                )

        
        # Control trigger and analyse + save data depending on toggles
        # Trigger will activate if triggered is True, FLASH button has been 
        # pressed, and trigger is not currently active
        if self.triggered == True and self.triggered_2 == False \
                                  and self.arm_trigger == True:
            print("Trigger on")

            # Code within this section will run once each time trigger is met
            #################################################################
            self.trigger_run()
            ################################################################# 

            # Set triggered active True
            self.triggered_2 = True                

        # Deactivate trigger if triggered is now False, FLASH button has been 
        # pressed and trigger is currently active
        elif self.triggered == False and self.triggered_2 == True \
                                     and self.arm_trigger == True:
            print("Trigger off")
            self.triggered_2 = False        
            self.arm_trigger = False
   

    def changePlotUnits(self):
        """
        Changes y axis range, label and plot title when units toggled
        """
        
        # Change units to FDC current depending on toggle input
        if self.ui.unitToggle.isChecked() == True:

            # Update y axis values to cover full range of signal, 
            # use min of y intercept and 0 to allow negative values
            y_min = min(self.FDC_conversion[1],0)
            y_max = (int(self.setup_params["Pico_max_voltage"])
                             * self.FDC_conversion[0] + self.FDC_conversion[1])
            self.ui.graphWidget.setYRange(y_min, y_max, padding=0)
            # Set text style (CSS styling)
            styles = {'color':'k', 'font-size':'10pt'}
            # Update graph labels
            self.ui.graphWidget.setLabel('left', 'Current (nA)', **styles)
            # Update toggle text
            self.ui.unitToggle.setText("Plot Units: Faraday Current (nA)")      

        # Change units to PicoScope voltage depending on toggle input
        else:

            # Y range goes from 0 to 5 V
            y_min = 0
            y_max = int(self.setup_params["Pico_max_voltage"])
            self.ui.graphWidget.setYRange(y_min, y_max, padding=0)
            # Set text style (CSS styling) 
            styles = {'color':'k', 'font-size':'10pt'}
            # Update graph labels        
            self.ui.graphWidget.setLabel('left', 'Voltage (mV)', **styles)
            # Update toggle text
            self.ui.unitToggle.setText("Plot Units: Picoscope Voltage (mV)")

    
    def Flash(self):
        """
        Runs on press of FLASH button, sends signal to Arduino to power FLASH 
        shutter for input time duration.

        Can run in two modes:

            a) Runs FLASH algorithm upon FLASh button pressed
            b) Waits for average beam current to match input, then runs FLASH
               algorithm

        FLASH algorithm stores beam current, then pauses data acquisition
        before sending flash duration to arduino. Pauses for long enough to 
        raise FD cup, flash and lower FD cup. Once flash time has been sent to 
        arduino, arms trigger in defined channel to capture pulse trace.
        Records beam current immediately after FD cup is lowered. While paused, 
        trigger should activate and pulse should be analysed. When finished, 
        checks pulse has been analysed (waits if necessary) and calculates 
        and displays outputs.
        
        """
        # Clear error message
        self.ui.FlashButtonError.setText("")
        self.ui.progressBar.setValue(0)
        self.ui.progressBar.setFormat('Initialising...')
        self.ui.progressBar.show()

        # Set button + style to disabled
        self.ui.flashButton.setStyleSheet("background-color:gray;" 
                                                "color: white;" "border: none")
        self.ui.flashButton.setEnabled(False)


        def run_FLASH():

            # Update progress bar
            self.ui.progressBar.setFormat('Flashing...')
            self.ui.progressBar.setValue(50)

            # Store faraday cup current before FLASH - average over current
            V_before = np.mean(self.FCD_data[:,1])
            current_before = (self.MK_conversion[1]
                                    + (V_before * self.MK_conversion[0]))
            
            # Pause data aquisition and change title of plot
            self.pause_plot = True
            title_text = "FARADAY CUP RAISED - DATA ACQUISTION PAUSED"
            self.ui.graphWidget.setTitle(title_text, color="r", size="14pt")

            # flash_time must be integer
            flash_time = int(self.ui.flashDurationEntry.text())

            # Write time to arduino in byte form
            data = f'{flash_time}\n'
            self.arduino.write(data.encode())
            print(f"Pulsing LED for {flash_time} milliseconds.")

            # Set FLASH boolean value to True to arm trigger
            self.arm_trigger = True

            # Wait for time for Faraday cup to open and close as per setup file
            QtTest.QTest.qWait(int(self.setup_params['FDC_wait_time_ms']))

            # NOTE: While waiting, trigger will activate, save file and analyse in background.

            # Resume plotting of data and reset plot title
            self.pause_plot = False
            self.ui.graphWidget.setTitle("Faraday Cup Current",
                                                     color="k", size="14pt")

            # Store faraday cup voltage and current after FLASH
            V_after = np.mean(self.FCD_data[:,1])
            current_after = (self.MK_conversion[1]
                                    + (V_after * self.MK_conversion[0]))

            # Set button enabled and set style to enabled
            self.ui.flashButton.setStyleSheet("background-color:red;" 
                                                    "color: white;")
            self.ui.flashButton.setEnabled(True)

            # Wait to output results if analysis has not finished
            while self.analysed == False:
                QtTest.QTest.qWait(50)
                
            # Finish and hide progress bar
            self.ui.progressBar.setValue(100)
            self.ui.progressBar.setFormat('Finalising...')

            # Calculate output parameters
            avg_V = (V_after + V_before) / 2
            avg_FD_current = (current_before + current_after) / 2
            avg_MK_current = (self.MK_conversion[1]
                        + avg_V * self.MK_conversion[0])
            avg_dose_rate = (avg_MK_current
                    * float(self.setup_params["Mk_chamb_dose_coefficient"]))

            total_dose = avg_dose_rate * self.pulse_duration

            # Update output labels, rounded to 3 d.p            
            self.ui.currentBeforeLabel.setText(
                f"Current before: {np.round(current_before, 3)} nA"
                )
            self.ui.currentAfterLabel.setText(
                f"Current after: {np.round(current_after, 3)} nA"
                )
            self.ui.avgCurrentLabel_2.setText(
                f"Average current: {np.round(avg_FD_current, 3)} nA"
                )
            self.ui.flashDurationLabel.setText(
                f"FLASH duration: {np.round((self.pulse_duration*1000), 3)} ms"
                )
            self.ui.doseDeliveredLabel.setText(
                f"Total dose delivered: {np.round(total_dose, 3)} Gy"
                )
            self.ui.avgDoseRateLabel.setText(
                f"Dose rate delivered: {np.round(avg_dose_rate, 3)} Gy/s"
                )
            
            # Show progress bar on 100% for 250ms before removing
            QtTest.QTest.qWait(250)
            self.ui.progressBar.hide()
        
        # Try except allows for error message to be displayed
        try:
            # Check whether program should wait for avg current to meet input
            if self.ui.waitforCurrentCheck.isChecked() == True:
                
                # Iterative counter for progress bar percentage
                progress_percentage = 0
                # Boolean value to check if smooth current found
                met = False

                # Attempt a maximum of 5 attempts to match current
                for runs in range(5):

                    # Empty list to hold measurements of current before pulse
                    measurements = []

                    # Take 10 measurements of current every 0.5s
                    for n in range(10):
                        measurements.append(self.FCD_data[-1,1] 
                        * (self.FDC_conversion[0]) + (self.FDC_conversion[1]))

                        # Wait 0.5s between measurements
                        QtTest.QTest.qWait(500)

                        # Show on progress bar that procesing has started
                        progress_percentage += 1
                        self.ui.progressBar.setValue(progress_percentage)
                        self.ui.progressBar.setFormat(
                            'Checking for smooth current')

                    # Calculate mean of measurements
                    avg_voltage = np.mean(measurements)
                    print(avg_voltage)

                    # Compare average to required current
                    req_voltage = float(self.ui.requiredCurrentEntry.text())

                    if (avg_voltage > (req_voltage
                    - int(self.setup_params['Req_current_thresh'])) 
                        and 
                    avg_voltage < (req_voltage
                    + int(self.setup_params['Req_current_thresh']))):
                        print("MET")
                        met = True
                        break
                
                # Run FLASH if current requirements have been met
                if met == True:

                    run_FLASH()

                # Do not run FLASH if current requirements have not been met
                else:

                    # Display error messages
                    self.ui.progressBar.setValue(0)
                    self.ui.progressBar.hide()
                    self.ui.FlashButtonError.setStyleSheet("color : red;" 
                                                                "font: 10pt")
                    self.ui.FlashButtonError.setText(
                                    "Current requirement could not be met.")

                    # Set button enabled and set style to enabled
                    self.ui.flashButton.setStyleSheet(
                                    "background-color:red;" "color: white;")
                    self.ui.flashButton.setEnabled(True)
                    # Finish and hide progress bar
                    self.ui.progressBar.setValue(100)
                    QtTest.QTest.qWait(100)
                    self.ui.progressBar.hide()

            # Run FLASH if no current requirements were set
            else:

                run_FLASH()

        # Handle errors
        except ValueError:
            # Display error messages
            self.ui.flashInputErrorLabel.setStyleSheet("color : red;" 
                                                        "font: 10pt")
            self.ui.flashInputErrorLabel.setText("Enter valid pulse time.")

        except AttributeError:
            self.ui.FlashButtonError.setStyleSheet("color : red;" "font: 10pt")
            self.ui.FlashButtonError.setText("Arduino not connected.")
            self.ui.progressBar.hide()
            self.ui.progressBar.setValue(0)


    def trigger_run(self):
        """
        Saves and analyses a snapshot of photodiode channel data upon trigger 
        being met. 

        Takes a snapshot of data from defined trigger channel. Saves data using
        numpy.savetxt. Runs flashpulseanalysis on pulse data to fit 
        antisymmetric logistic curves to left and right sides. Then evaluates 
        rise time, fall time and total pulse duration. Saves these data and 
        produces plot showing scattered raw data and calculated fit. Updates 
        'analysed' attribute when complete so other parts of program can stop 
        waiting for analysis to finish. If user has specified, saves plot as 
        png image.

        """ 

        # Set data of scatter plot
        self.scatter.setData(self.FCD_data[:,0], self.FCD_data[:,1])
        # Show scatter plot in new tab
        self.ui.plotTabs.setTabVisible(1, True)

        # Update progress bar
        self.ui.progressBar.setFormat('Saving data...')
        self.ui.progressBar.setValue(65)

        # Save data to disk
        # Get current date + time for unique file name
        now = datetime.now().strftime("%Y.%m.%d_%H%M%S")
        f_name = f"Measurements/FLASH_Measurement_{now}.txt"
        np.savetxt(f_name, self.laser_data)

        # NOTE: Need to decide which is fastest way of saving data. Numpy, 
        # pandas? Also can, it be done in another thread?

        # Update progress bar
        self.ui.progressBar.setFormat('Analysing pulse...')
        self.ui.progressBar.setValue(85)


        # NOTE: RUN ANALYSIS ON CAPUTURED DATA
        
        # Update attribute to show analysis has completed
        self.analysed = True

        # Define variables / attributes calculated from fit
        fit_x = None
        fit_y = None
        self.pulse_duration = 0
        self.rise_time = 0
        self.fall_time = 0

        # Save plot of fitted pulse if toggled
        if self.ui.savePulseCheck.isChecked() == True:

            try:
            
                plt.figure()
                plt.title(f"Measurements/FLASH_Measurement_{now}")
                plt.scatter(self.laser_data[:,0], self.laser_data[:,1], 
                                                    marker='x', label='Data')
                plt.plot(fit_x, fit_y, color='red', label='Fit')
                text_y = 1500
                text_x = 1000
                print(text_x, text_y)
                round=6
                plt.text(text_x, text_y, 
                f'Pulse duration: {np.round(self.pulse_duration, round)} s\
                    \nRise time: {np.round(self.rise_time, round)} s\
                    \nFall time: {np.round(self.fall_time, round)} s',
                     ha='center', fontsize = 14)
                plt.legend(loc='best')
                plt.savefig(f"Measurements/FLASH_Measurement_{now}.png")
                plt.clear()
            
            except :
                print("Error saving figure.")

        print("Trigger routinte complete")


    def tab_add(self):
        """
        Adds row to calibration table.
        """

        row_n = self.ui.calibTable.rowCount()
        self.ui.calibTable.insertRow(row_n)

    
    def tab_remove(self):
        """
        Removes row from calibration table.
        """

        row_n = self.ui.calibTable.rowCount()
        self.ui.calibTable.removeRow(row_n-1)


    def calib_save(self):
        """
        Produces file explorer popup that allows user to save calibration data.
        """
        try:
            # Get number of rows in GUI table
            row_n = self.ui.calibTable.rowCount()
            # Create empty array of same size as GUI table
            save_data = np.zeros((row_n, 2))
            # Copies each item from GUI table into array
            for n in range(row_n):
                # Need to convert text from table to float
                save_data[n,0], save_data[n,1] = \
                                float(self.ui.calibTable.item(n, 0).text()), \
                                float(self.ui.calibTable.item(n, 1).text())
            
            # Path to folder save calibration text files to
            save_path = "C:/Users/maxco/OneDrive - University of Birmingham/ \
            Summer Internship/FLASH Shutter Project/Final Files/Calibrations"
            # QFileDialog that allows user to define a file name & location
            name = QFileDialog.getSaveFileName(self, 'Save File', save_path,
                                                        "Text files (*.txt)")
            # Save text file
            np.savetxt(name[0], save_data)

        except AttributeError:
            print("Check all fields are populated, delete any empty rows.")

        except FileNotFoundError:
            pass
    
        except:
            print("Other error")


    def calib_load(self):
        """
        Produces file explorer popup that allows user to select calibration 
        data from file.
        """
        
        try:
            # Path to folder to load calibration files from
            load_path = "C:/Users/maxco/OneDrive - University of Birmingham/ \
            Summer Internship/FLASH Shutter Project/Final Files/Calibrations"
            # QFileDialog that allows user to define a file name & location
            name = QFileDialog.getOpenFileName(self, 'Open File', load_path, 
                                                        "Text files (*.txt)")
            # Get text file
            load_data = np.loadtxt(name[0])
            # Determine number of rows in text file
            n_rows = np.shape(load_data)[0]
            # Set size of tabel and add data fom each row to GUI table
            for row in range(n_rows):
                self.ui.calibTable.setRowCount(n_rows)
                column1 = QTableWidgetItem(str(load_data[row,0]))
                self.ui.calibTable.setItem(row, 0, column1)
                column2 = QTableWidgetItem(str(load_data[row,1]))
                self.ui.calibTable.setItem(row, 1, column2)

        except FileNotFoundError:
            pass

        except:
            print("Other error")


    def calibration(self):
        """
        Uses scipy's curve fit function to fit linear trend line to calibration 
        data.

        """
        # Clear error message
        self.ui.calibrationErrorLabel.setText("")

        # Define linear fit
        def linear(x, m, c):

            return m*x + c

        # Attempt to fit linear curve to data
        try:
            # Get size of GUI table, create array of same size to store data
            row_n = self.ui.calibTable.rowCount()
            calib_data = np.zeros((row_n, 2))
            # Get data from table by iterating through each item in each row
            for n in range(row_n):

                calib_data[n,0], calib_data[n,1] = \
                        float(self.ui.calibTable.item(n, 0).text()), \
                        float(self.ui.calibTable.item(n, 1).text())

            # Use scipy's curve_fit function to fit linear curve to data
            popt, pcov = curve_fit(linear, calib_data[:,0], 
                                    calib_data[:,1], p0=[1,1])
            
            # Update labels to show calibration results
            p0 = np.round(popt[0], 2)
            p1 = np.round(popt[1], 2)
            self.ui.calibrationText.setText(
                f'Calibration: y = {p0}x + {p1}')
            self.ui.calibratedErrorLabel.setText(
                f'Calibration: y = {p0}x + {p1}')
            
            # If user has selected to plot calibration, do so
            if self.ui.viewCalibPlotBool.isChecked() == True:
                x_smooth = np.linspace(np.min(calib_data[:,0]), 
                                            np.max(calib_data[:,0]), 1000)
                plt.figure()
                plt.title("Calibration Plot")
                plt.scatter(calib_data[:,0], calib_data[:,1], marker='x', 
                                                        label="Table Data")
                plt.plot(x_smooth, linear(x_smooth, *popt), color='red', 
                        label = f"Linear Fit: y = {p0}x + {p1}")
                plt.xlabel("Picoscope Voltage (mV)")
                plt.ylabel("Markus Chamber Dose Rate (Gy/s)")
                plt.legend(loc='upper left')
                plt.show()

            # Set boolean calibrated value to True, update calibration values 
            # to gradient and y intercept of calibration
            self.calibrated = True
            self.MK_conversion = popt

        # Update error label if there are blank spaces left in table
        except AttributeError:
            self.ui.calibrationErrorLabel.setStyleSheet("color : red;" 
                                                        "font: 10pt")
            self.ui.calibrationErrorLabel.setText(
                "Check all fields are populated, delete any empty rows.")
        # Catch any other errors
        except:
            self.ui.calibrationErrorLabel.setStyleSheet("color : red;" 
                                                        "font: 10pt")
            self.ui.calibrationErrorLabel.setText(
                "Issue with data entered, could not calibrate.")


    def calculateFlashParams(self):
        """
        Calculates and updates required parameter labels, if calibrated.

        Should run on press of 'Calculate' button to get entered dose and dose
        rate and calculate required current, picoscope voltage and FLASH 
        duration. Only runs if system is calibrated, otherwise calculations are
        not possible.
        """
        # Reset error label
        self.ui.flashInputErrorLabel.setText("")

        if self.calibrated == True:

            try:
                # Get numerical inputs from text entries
                required_dose = float(self.ui.doseEntry.text())
                required_dose_rate = float(self.ui.doseRateEntry.text())
                # Calculate flash time (convert to milliseconds)
                f_time = required_dose * 1000 / required_dose_rate
                # Calculate required current
                calc_current = (((required_dose 
                    / float(self.setup_params['Mk_chamb_dose_coefficient'])) 
                    - self.MK_conversion[1]) 
                    / self.MK_conversion[0])
                # Convert required current to required Picoscope voltage
                calc_voltage = ((calc_current - self.FDC_conversion[1]) 
                                / self.FDC_conversion[0])
                # Update labels with results
                self.ui.requiredVoltageEntry.setText(str(np.round(calc_voltage,3)))
                self.ui.requiredCurrentEntry.setText(str(np.round(calc_current,3)))
                self.ui.flashDurationEntry.setText(str(int(f_time)))
            
            except ValueError:
                # Output error
                self.ui.flashInputErrorLabel.setStyleSheet("color : red;" 
                                                            "font: 10pt")
                self.ui.flashInputErrorLabel.setText(
                    "Please enter only numerical values.")


        
        else:
            # Output error
            self.ui.flashInputErrorLabel.setStyleSheet("color : red;" 
                                                        "font: 10pt")
            self.ui.flashInputErrorLabel.setText("Please calibrate system.")

    

    
    def showHelp(self):
        """
        Creates an instance of the help popup window and displays it.
        """
        self.help = helpPopup()
        self.help.show()


    def close(self):
        """
        Closes the application.
        """
        app.quit()


class helpPopup(QMainWindow):

    def __init__(self):
        """
        Initialises the help popup, filling the main scroll area with text from 
        file: 'help.txt'.
        """
        super().__init__()

        self.ui = Help_MainWindow()
        self.ui.setupUi(self)
        f = open("help.txt", "r")
        self.ui.userGuideText.setText(f.read())
        f.close()
        self.setWindowTitle("FLASH Control Panel User Guide")        


if __name__ == "__main__":
    # Create the application
    app = QApplication(sys.argv)
    # Create and show the application's main window
    win = Window()
    win.setWindowTitle("FLASH Control Panel")
    win.setWindowIcon(QtGui.QIcon("flash_icon.png"))
    win.show()
    # Run the application's main loop
    sys.exit(app.exec())