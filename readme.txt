To run the controller:

Make sure that the controller is in the correct controller path according to the requirements of Webots

Make sure that the world is also in the correct world path according to the requirements of Webots

Make sure that all other folders are on the Python path. This can be done e.g. by editing the bashrc

In the controller, make sure that the path the human and controller configuration files are edited correctly. (Lines 51/53/77-88)

In line 46, you can pick the controller that you want to run.

Note that the lines related to logging are commented out by default.

If you have access to the BIOROB optimizer (see biorob.epfl.ch), you can run optimizations by changing self.opti in line 71/72.

MovingGroundfull is the controller of the ground perturbation. The index 53719 was used for the paper. Index 1 is the beginning of the signal. This controller should also be in the controller path according to the Webots requirements
