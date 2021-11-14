# Robot learning techniques for set point regulation of robots with nonlinear flexibility on the joints

This is our Final Project for the course in Underactuated Robotics A.Y. 2020/2021.

The task has been to combine learning techniques with Nonlinear MPC in order to perform set-point regulation of a manipulator with **elastic joints**. In particular, the elasticity term is assumed to be **nonlinear** and **unknown** (hence the need of the learning techniques).

## Project structure

Here we summarize the project structure for ease of use.

- `simulation.m`: main file of the project
- `simulationOnline.m`: main file of the project used to run the simulations with online retraining
- `parameters.m`: collects all the parameters across the project
- `robotModel.m`: derives symbolically (and saves numerically as MATLAB functions) the needed terms of the robot dynamical model
- `mpcSetup.m`: sets up the MPC object
- `plotResults.m`: displays the results of the simulations

Some utility functions are used.

- `dataGeneration`: collects all the utilities for generating offline training data
- `modelFunctions`: collects the functions for the terms of the robot dynamical model
- `modelsTraining`: collects all the utilities for training the learning models
- `mpcFunctions`: collects all the functions for MPC (prediction model, custom cost function, custom constraints)
- `savedData`: collects generated datasets and trained models
- `utils`: collects various utility functions used across the project


## Documentation

Check the report of the project [here](documentation/report.pdf) and the [presentation](documentation/presentation.pptx) we gave of our project.

## Resources used
- MATLAB

## Authors
- Andrea Caciolai
- Emanuele Nicotra
- Matilde Rabbiolo
