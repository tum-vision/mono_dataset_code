


=================== INSTALL ========================

0. install eigen:
sudo apt-get install libeigen3-dev

1. install ziplib:
sudo apt-get install zlib1g-dev
unzip thirdparty/libzip-1.1.1
cd thirdparty/libzip-1.1.1
./configure
make
sudo make install
sudo cp lib/zipconf.h /usr/local/include/zipconf.h   (no idea why that is needed).


2. install aruco marker detection:
see eg here: http://maztories.blogspot.de/2013/07/installing-aruco-augmented-reality.html
we tested with version 1.3.0. which is included in /thirdparty.




2. Build
cmake . && make





=================== USE ========================

====== playbackDataset: =========
Shows images of a dataset. Meant as example code regarding how to read the dataset.
Run with "./playDataset X/sequence_01/" (mind the trailing slash)!

optionally, the calibration is used for 
	* rectification (r)
	* response function inversion (g)
	* vignette removal (v)
	* removal of over-exposed (white) images. (o).
Pressing the respective key toggles the option.
See code for details.


====== responseCalib: =========
Performs photometric calibration from a set of images, showing the exact same scene at different exposures.
Run with "./responseCalib X/CalibrationDatasets/narrow_sweep1/" (mind the trailing slash)!
outputs some intermediate results, and pcalib.txt containing the calibrated inverse response function to photoCalibResult.
See code for details.


====== vignetteCalib: =========
Performs photometric calibration from a set of images, showing a flat surface with an ARMarker.
Run with "./vignetteCalib X/CalibrationDatasets/narrow_vignette/" (mind the trailing slash)!
outputs some intermediate results, and vignette.png (16-bit png) containing the calibrated vignette function.
vignetteSmoothed.png is a slightly smoothed version, mainly to remove the black borders (pixels at the border are never observed).
See code for details.
WARNING: requires a lot of Memory (16GB ram for 1000 input images)! Can easily be changed at 
the cost of slightly slower runtime... you'll have to do that yourself.

