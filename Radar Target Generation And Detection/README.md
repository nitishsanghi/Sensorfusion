# RADAR Target Generation and Detection

In this project a RADAR target detection is simulated by generating a target signal and then using 1D FFT, 2D FFT, and constant false alarm rate (CFAR) to detect the range of the simulated target and the velocity. A Frequency Modulated Continuous Wave (FMCW) RADAR signal is simulated and the return signal is processed for frequency shift due to range and doppler effect. 

A 2D FFT of the mixer signal to detect range and velocity of a target. As can be seen there is a lot of noise in the signal and requires some filtering to extract a reliable estimate.
<img src="https://github.com/nitishsanghi/Sensorfusion/blob/master/Radar%20Target%20Generation%20And%20Detection/images/2D_FFT.jpg" width="820" height="410" />

A 2D FFT CFAR thresholded of the mixer signal to detect range and velocity of a target. 
<img src="https://github.com/nitishsanghi/Sensorfusion/blob/master/Radar%20Target%20Generation%20And%20Detection/images/2D_FFT_CFAR.jpg" width="820" height="410" />
