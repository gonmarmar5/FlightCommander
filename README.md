# FlightCommander.py 🚁

## Requirements

- Python v3.6 or greater
- V-REP 3.6.2 or greater
- Linux 64x

## Quick Start 🏁

- **Make sure you have Python 3.6 or above installed**

  - `python3 --version`


- **Install Python from Pyenv**

  - Install [pyenv](https://mrdjangoblog.wordpress.com/2016/08/18/installing-pyenv-python-3-5/)
  - `pyenv install 3.6.0`

- **How to install** 

  - Git `clone` this repo or download as a ZIP and extract
  - Run `python main.py`

- **Run CoppeliaSym**

  - Go to `~/Descargas/CoppeliaSim_Edu_V4_6_0_rev10_Ubuntu20_04`
  - Run `sudo ./coppeliaSim.sh`
  - Open the scene `chanllenge_scenario.ttf`
  - Set Dynamics Engine to `Bullet 2.7`
  - Start simulation

- **Extra Configuration** ⚙️

  - To run `quadricopter-py` on other operating systems you need to replace the `remote_api.so` located in the folder `../V-REP_PRO_EDU_V3_6_2/programming/remoteApiBindings/lib/lib`.

## Documentation

- **Class SceneMap**
    - Define the limits of the quadricopter simpler.

- **Class Quadricopter**
    - Represents the drone with its respective attributes and sensors.

- **Class VisionSensor**
    - Responsible for Quadricopter vision

- **Class TargetControl**
    - Object responsible for move the quadricopter. Him works like a control remote. 

- **Class SonarSensor**
    - Responsible for detect colision of the quadricopter. Proximity sensor.

## Wiki

- [GitHub](https://github.com/gonmarmar5/FlightCommander/wiki/)