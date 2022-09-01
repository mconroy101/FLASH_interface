import numpy as np
from scipy.optimize import curve_fit


class SideAnalyser:
    
    def __init__(self, side, signal, approx_width):

        # Required attributes
        self.side = side
        self.approx_width = approx_width
        self.fall_time = 0
        self.rise_time = 0
        self.pulse_duration = 0
        self.time = signal[:,0]
        self.voltage = signal[:,1]


    def asym_logistic(self, x, m, s, N, c, v):
        """
        Function for an asymmetric logistic function.

        Args:
            x (arr: Float): x data values to feed into function
            m (Float): Equivalent "mean" parameter
            s (Float): EquivaLent "variance" parameter
            N (Float): Normalisation parameter
            c (Float): Constant y shift parameter
            v (Float): Asymmetry parameter

        Returns:
            y (arr: Float): y values of function
        """
        y = N / (1 + np.exp(-(x-m)/s))**(1/v) + c
        
        return y


    def select_range(self, start=0):
        """
        Locates approximate location of rising or falling edge in FLASH signal.
        
        Identifies indices of rise and fall of peak by calculating difference
        between every Nth point. For differences > defined noise threshold, 
        store index in array. Difference between max and min index in array is
        estimate of pulse width. Median value of array is estimate of centre of
        rising/falling side. Applies cuts to data to cover 5 times estimated 
        pulse width, either side of central estimate.      
        
        Args:
            start (int, optional): Index from which to start counting. If 
            searching for second edge of pulse, set to index of first pulse to 
            prevent searching data that is know. Defaults to 0.
        """

        # Boolean value of whether pulse width is unacceptable
        pw_bad = True

        # Threshold above which values of diff are considered signal 
        # Needs adjusting depending on data set
        noise_thresh = 10 

        # While loop allows for iteration through increasing values for noise in data
        while pw_bad == True:
            
            # Calculate differnence between every Nth point, defined below
            search_width = 9 
        
            # Find difference between every nth point in voltage array
            delta = np.diff(self.voltage[start::search_width])

            # If searching for left side...
            if self.side == "left":

                # Find indices of all points where change in signal is above 
                # threshold, adjust for sampling every nth element
                pulse_loc = ((np.where(delta > noise_thresh)[0]) * search_width
                                                + search_width - 1 + start)
                # Find approximate width of pulse
                pulse_width = pulse_loc[-1] - pulse_loc[0]
                # Too large a pulse width indicates that noise is being 
                # detected as signal, so increase noise threshold if > 120
                if int(pulse_width) < 120:
                    pw_bad = False  # Pulse width OK
                else:
                    # Increase noise threshold to reduce noise in signal
                    noise_thresh += 5
                # Find approximate index of central (steepest) point of pulse
                centre = int(np.round(pulse_loc[0] + pulse_width / 2))
                
                # Fit and plot over approximately 6 times pulse width
                step = 6*pulse_width
                # Cut data with extra data before to get better low tail fit
                self.t_cut = self.time[centre-2*step:centre+step]
                self.v_cut = self.voltage[centre-2*step:centre+step]

            # If searching for right side...
            elif self.side == "right":

                # Find indices of all points where change in signal is above 
                # threshold, adjust for sampling every nth element
                pulse_loc = (np.where(delta < -noise_thresh)[0])*search_width + search_width - 1 + start

                # Find approximate width of pulse
                pulse_width = pulse_loc[-1] - pulse_loc[0]
                # Too large a pulse width indicates that noise is being 
                # detected as signal, so increase noise threshold if > 120
                if pulse_width < 120:
                    pw_bad = False
                else:
                    # Increase noise threshold to reduce noise in signal
                    noise_thresh += 5
                # Find approximate index of central (steepest) point of pulse
                centre = int(np.round(pulse_loc[0] + pulse_width / 2))

                # Fit and plot over approximately 6 times pulse width
                step = 6*pulse_width
                # Cut data with extra data after to get better low tail fit
                self.t_cut = self.time[centre-step:centre+2*step]
                self.v_cut = self.voltage[centre-step:centre+2*step]

        # Index where edge has been estimated to be found
        self.sig_mid_est = centre
        self.pulse_lims = pulse_loc[0], pulse_loc[-1]


    def analyse_side(self):
        """
        Uses scipy.optimise curve_fit to fit asymetric logistic curve to data.
        
        """

        # Different initial guess parameters & bounds for LHS vs RHS
        if self.side == "left":
            # Fit using scipy optimise
            self.popt, pcov = curve_fit(self.asym_logistic,
                self.t_cut, self.v_cut,
                p0=(0, -0.0004, -250, 393, 5),
                bounds=[[-np.inf, -np.inf, -np.inf, -np.inf, 1],
                        [np.inf, np.inf, np.inf, np.inf, 10]])
            # Calculate error
            self.perr = np.sqrt(np.diag(pcov))

        if self.side == "right":
            # Fit using scipy optimise
            # Use approx_width input from user as best guess for m parameter
            self.popt, pcov = curve_fit(self.asym_logistic,
                self.t_cut, self.v_cut, 
                p0=(self.approx_width, 0.0002, -250, 393, 5), 
                bounds=[[0, -np.inf, -np.inf, -np.inf, 1], 
                        [np.inf, np.inf, np.inf, np.inf, 10]])
            # Calculate error
            self.perr = np.sqrt(np.diag(pcov))

        # Extract important parameters from fit with errors
        m = self.popt[0]
        mean_err = self.perr[0]
        s = self.popt[1]
        s_err = self.perr[1]
        N = self.popt[2]
        N_err = self.perr[2]
        c = self.popt[3]
        c_err = self.perr[3]
        v = self.popt[4]
        v_err = self.perr[4]


        # Calculate important information
        # Uses solution of setting asym_logistic fn = 0.1N / 0.5N / 0.9N
        self._50 = m - s*np.log((0.5)**(-v)-1)
        self._10 = m - s*np.log((0.1)**(-v)-1)
        self._90 = m - s*np.log((0.9)**(-v)-1)
        # Calculate rise time
        self.rise_time = np.abs(self._90 - self._10)

            
class PulseAnalysis:

    def __init__(self, data_in, approx_width):
        """
        Method that fits both sides of pulse and combines results to give full 
        fit of pulse. Outputs required measurements.

        Args:
            data_in (2D arr: Float): Nx2 array of time and voltage data to fit
            approx_width (Float): Approximate time of right edge of pulse
        """

        # Initialise objects for left and right side of pulse
        left = SideAnalyser("left", data_in, approx_width)
        right = SideAnalyser("right", data_in, approx_width)
        
        # Perform analysis on left side of pulse
        left.select_range()
        left.analyse_side()
        # Perform analysis on right side of pulse, starting at location of left
        right.select_range(left.sig_mid_est)
        right.analyse_side()

        # Calculate duration of pulse and set accessible attributes
        self.p_duration = right._50 - left._50
        self.rise_time = left.rise_time
        self.fall_time = right.rise_time

        # Produce smooth x values to plot
        L_curve_x = np.linspace(left.t_cut[0], left.t_cut[-1], 10000)
        R_curve_x = np.linspace(right.t_cut[0], right.t_cut[-1], 10000)

        # Produce logistic values using smooth points above
        L_curve_y = self.asym_logistic(L_curve_x, *left.popt)
        R_curve_y = self.asym_logistic(R_curve_x, *right.popt)

        # Combine left and right curve data
        fit_x = np.concatenate((L_curve_x, R_curve_x))
        # Insert value for straight line before
        fit_x = np.insert(fit_x, 0, left.time[0])  
        # Insert value for straight line after   
        fit_x = np.append(fit_x, left.time[-1])       

        # Again for y values
        fit_y = np.concatenate((L_curve_y, R_curve_y))
        fit_y = np.insert(fit_y, 0, L_curve_y[0])
        fit_y = np.append(fit_y, R_curve_y[-1])

        # Define tuple attributes that can be accessed for plotting of data
        self.pulse_scatter = left.time, left.voltage
        self.left_scatter = left.t_cut, left.v_cut
        self.right_scatter = right.t_cut, right.v_cut

        self.pulse_fit = fit_x, fit_y
        self.left_fit = L_curve_x, L_curve_y
        self.right_fit = R_curve_x, R_curve_y
        

    def asym_logistic(self, x, m, s, N, c, v):
        """
        Function for an asymmetric logistic function.

        Args:
            x (arr: Float): x data values to feed into function
            m (Float): Equivalent "mean" parameter
            s (Float): EquivaLent "variance" parameter
            N (Float): Normalisation parameter
            c (Float): Constant y shift parameter
            v (Float): Asymmetry parameter

        Returns:
            y (arr: Float): y values of function
        """

        y = N / (1 + np.exp(-(x-m)/s))**(1/v) + c
        
        return y