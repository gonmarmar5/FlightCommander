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
- **Quadricopter**
    - Represents the drone with its respective attributes and sensors.

- **SceneMap**
    - Fundamental for define the limits of the quadricopter simpler.

- **VisionSensor**
    - Responsible for Quadricopter vision

| Atribute             | Type      |                Description                           | Default value |
|:--------------------:|:---------:|:----------------------------------------------------:|:-------------:|
| id                   | int       | Get object handle vision sensor                      |    |
| _clientID            | int       | Id of the client with api remote                     | |
| resolution           | int       | Resolution the image                                 | |
| line                 | int       | Vector line representing a matrix line               | |
| half                 | int       | Divide the image into two parts                      | |  

| Method               | Parameters|                Description                           |        Return                    |
|:--------------------:|:---------:|:----------------------------------------------------:|:--------------------------------:|
| getImage             |           | Get imagem of the vision sensor                      | Return an array with view values       |
| getPositionObject    | image - Array of the captured image  <br> refObj - Object value reference in vision sensor   | Get the object position of the vision sensor                      | Return an array with orientation and direction respectively |

- **TargetControl**
    - Object responsible for move the quadricopter. Him works like a control remote. 

| Atribute             | Type      |                Description                           | Default value |
|:--------------------:|:---------:|:----------------------------------------------------:|:-------------:|
| id                   | int       | Get object handle vision sensor                      |    |
| _clientID            | int       | Id of the client with api remote                     | | 

| Method               | Parameters|                Description                           |        Return                    |
|:--------------------:|:---------:|:----------------------------------------------------:|:--------------------------------:|
| getPosition          |           | Get the target position                              | Return the coordinate of the target x, y, z |
| setPosition          | x, y, z   | Set the target position                              |  |

- **SonarSensor**
    - Responsible for detect colision of the quadricopter. Proximity sensor.

| Atribute             | Type      |                Description                           | Default value |
|:--------------------:|:---------:|:----------------------------------------------------:|:-------------:|
| id                   | int       | Get object handle sonar  sensor                      |    |
| _clientID            | int       | Id of the client with api remote                     |    | 

| Method               | Parameters|                Description                           |        Return                    |
|:--------------------:|:---------:|:----------------------------------------------------:|:--------------------------------:|
| getStateColision     |           | Checks if the quadricopter will collide              | Return true if will collide and false if not |

## Wiki

- [GitHub](https://github.com/gonmarmar5/FlightCommander/wiki/)