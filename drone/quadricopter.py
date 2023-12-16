import time

from util import vrep
from scene.scene_map import SceneMap

class Quadricopter():
    ''' PARAMETERS:
    
    _serverIp (str): Address ip of the server remote. 
    _serverPort (int): Port of the server remote.
    _refObj(int): Object value reference in vision sensor
    _clientID (int): Id of the client with api remote
    target (object): Target that control the quadricopter
    vision (object): Vision that get image
    sonar (object): Responsible for detecting collision when landing
    vMin (float): Minimum speed while searching.
    _objFound (bool): State object found
    msg (str): Mensage of the quadricopter
    '''

    _serverIp   = None
    _serverPort = None
    _refObj     = None
    _clientID   = None
    target      = None
    vision      = None
    sonar       = None
    vMin        = None
    _objFound   = None
    msg         = ''

    def __init__(self, refObj, _serverIp:str = '127.0.0.1' , _serverPort:int = 19999, vMin:float = 0.05):
        self._serverIp     = _serverIp
        self._serverPort   = _serverPort  
        self._refObj       = refObj
        self.vMin          = vMin
        self._clientID     = self._startServer()
        self.target        = self._createTargetControl()
        self.vision        = self._createVisionSensor()
        self.sonar         = self._createSonarSensor()
    
    def _createTargetControl(self):
        '''
        Goal: Get object handle that control Quadricopter 
        '''
        return Quadricopter.TargetControl(self._clientID)

    def _createVisionSensor(self):
        '''
        Goal: Get object handle that control Vision Sensor
        '''
        return Quadricopter.VisionSensor(self._clientID)

    def _createSonarSensor(self):
        '''
        Goal: Get object handle that control Sonar Sensor
        '''
        return Quadricopter.SonarSensor(self._clientID)

    def _startServer(self) -> bool:
        '''
        Goal: Start api remote 

        Return: The client ID, or -1 if the connection to the server was not possible
        '''
        return vrep.simxStart(self._serverIp, self._serverPort, True, True, 2000,5)

    def _finishServer(self):
        '''
        Goal: Close the comunication with api remote
        '''
        vrep.simxFinish(self._clientID)

    def startPosition(self, vStart, sMap: SceneMap):
        '''
        Goal: Adjust the coordinates of the dron to the initial position

        Input:
        - vStart(float): Speed of the dron
        - sMap(SceneMap): Map of the scene

        Output:
        - None
        '''
        pos = self.target.getPosition()
        x = sMap.xMin
        y = sMap.yMin
        z = sMap.zMax
        
        while pos[0] > x or pos[1] > y or pos[2] < z:
            # Adjust coordinates of the dron to the initial position
            if pos[2] < z:
                xNew = pos[0]
                yNew = pos[1]
                zNew = pos[2] + vStart
            else:
                xNew = pos[0] - vStart if pos[0] > x else pos[0]
                yNew = pos[1] - vStart if pos[1] > y else pos[1]
                zNew = pos[2]
            
            self.target.setPosition(xNew, yNew, zNew)
            time.sleep(0.05)
            
            # If the object is found while adjusting the position, it is not necessary to continue
            image = self.vision.getImage()
            if self._refObj in image:
                self._objFound = True
                self.vMin = self.vMin/50
                break
            
            # This allows the loop to adjust the coordinates of the dron
            pos = self.target.getPosition()
            
    def searchObj(self, sMap: SceneMap):
        '''
        Goal: Search for the object in the specified map.

        Input:
        - sMap (SceneMap): The map of the scene.

        Parameters:
        - v (float): Velocity variable for drone movement.
        - y_control (int): Counter for controlling y-axis movement.
        - xEnable (bool): Flag to enable/disable x-axis movement.
        - yEnable (bool): Flag to enable/disable y-axis movement.
        - boss (bool): Flag indicating whether the drone is in "boss" mode.

        Output:
        - None
        '''

        v = 0
        y_control = 0
        xEnable = True
        yEnable = False
        boss = False
        
        while vrep.simxGetConnectionId(self._clientID) != -1:
            x, y, z = self.target.getPosition()
            
            if x >= (sMap.xMin) and y >= (sMap.yMax):
                self.msg = 'Drone out of bounds!'
                break 
            
            if xEnable:
                # Change direction when reaching xMin or xMax
                if (x < sMap.xMin and self.vMin < 0) or (x > sMap.xMax and self.vMin > 0):
                    self.vMin = self.vMin * (-1)
                    
                    xEnable = False
                    yEnable = True
                
                # Activate boss mode when the drone is in the border area, otherwise deactivate it
                if (x > sMap.xMin+1 and x < sMap.xMax-1 and not boss):
                    boss = True
                else:
                    if (x <= sMap.xMin+1 or x >= sMap.xMax-1 and boss):
                        boss = False
                
                # This increase the velocity when the drone is in "boss" mode
                v = self.vMin*4 if boss else self.vMin  
                x = x + v
            
            if yEnable:
                y = y + 0.1
                y_control += 1

                # Check if the drone has moved 4 units in the y-axis
                if (y_control >= 40):
                    y_control = 0
                    yEnable = False
                    xEnable = True
            
            self.target.setPosition(x, y, z)

            if self.sonar.getStateColision():
                print("I'm gonna collide?: " ,self.sonar.getStateColision())
                self.vMin = 0
                #todo move the drone to a position where it is not in danger of colliding
                
            time.sleep(0.1)

            #todo: Tengo que ver como sabe que el objeto esta en la imagen
            image = self.vision.getImage()
            if self._refObj in image:
                self._objFound = True
                self.vMin = self.vMin/50
                return
    
        self._objFound = False
        self.msg = 'Object not found!'

    def land(self, sMap: SceneMap):
        '''
        Goal: Safely land the drone on the specified map.

        Input:
        - sMap (SceneMap): The map of the scene.

        Parameters:
        - height (float): Variable to control the descent speed of the drone.
        - notFound (int): Counter for tracking the number of consecutive frames where the object is not found.

        Output:
        - bool: True if the drone has successfully landed on the specified map, False otherwise.
        '''

        height = 0  
        notFound = 0  

        while True:
            x, y, z = self.target.getPosition()

            # Check if the drone is in or below the specified minimum height
            if z <= sMap.zMin:
                self.msg = "Find the object! It is below."
                return True

            image = self.vision.getImage()
            orientation, direction = self.vision.getPositionObject(image, self._refObj)

            if orientation != -1 and direction != -1:
                if orientation == 0 and direction == 0 and z <= sMap.zMin:
                    self.msg = "Find the object! It is below."
                    return True 

                # Adjust the descent speed based on object orientation and drone altitude
                if orientation == 0 or direction == 0:
                    height = -0.01

                if z <= sMap.zMax/2:
                    height = height * 10
                    self.vMin = self.vMin / 10
                
                # Move the drone to a new position based on object position and altitude adjustment
                if not self.sonar.getStateColision():
                    self.target.setPosition((x + orientation), (y + direction), (z + height))
                else:
                    self.msg = "Find the object! It is below. Dangerous airfield!"
                    return True
                        
                # Reset variables for the next iteration
                height = 0
                notFound = 0
            else:
                # If object position is not identified, move the drone in a default direction
                self.target.setPosition((x - self.vMin), y, z)
                notFound += 1

            # Check if the object is lost for an extended period
            if notFound > 500:
                self.msg = "Lost object"
                self.vMin = self.vMin * 10
                return False

            time.sleep(0.15)

    class VisionSensor():
        ''' PARAMETERS:

        id (int): Get object handle vision sensor
        _clientID (int): Id of the client with api remote
        resolution (int): Resolution the image
        line (int): Vector line representing a matrix line
        half (int): Divide the image into two parts
        '''

        id          = None
        _clientID   = None
        resolution  = None
        line        = None
        half        = None


        def __init__(self, clientID):

            # Get the object handle for the vision sensor
            self.id = vrep.simxGetObjectHandle(clientID, 'Vision_sensor', vrep.simx_opmode_blocking)[1]
            self._clientID = clientID

            # Initiate image streaming for the vision sensor
            vrep.simxGetVisionSensorImage(clientID, self.id, 0, vrep.simx_opmode_streaming)
            time.sleep(0.5)

        def getImage(self):
            '''
            Goal: Get the image from the vision sensor.

            Output: 
            - image (list), a list representing the image captured by the vision sensor (each element represents a pixel value).
            '''

            if not self.resolution:
                # Initiate image retrieval in buffer mode. 
                # NOTE: using buffer mode ensures that the most recent vision sensor image is retrieved when the method is called.
                error, res, image = vrep.simxGetVisionSensorImage(self._clientID, self.id, 0, vrep.simx_opmode_buffer)
                
                self.resolution = res[0]
                self.line = self.resolution * 3
                self.half = int(self.line * self.resolution / 2)
                
                return image
            
            return vrep.simxGetVisionSensorImage(self._clientID, self.id, 0, vrep.simx_opmode_buffer)[2]

        def getPositionObject(self, image, refObj) -> list:
            '''
            Goal: Get the object position of the vision sensor, used in landing.

            Input: 
            - image (list): Array of the captured image.
            - refObj (int): Object value reference in vision sensor.
            
            Output: 
            - An array with orientation and direction respectively.
            '''

            def search_in_half(half, refObj, orientation):
                '''
                Goal: Search for the object in a given image half.

                Input:
                - half (list): Image half to search.
                - refObj (int): Object value reference in vision sensor.
                - orientation (float): Current orientation.

                Output: 
                - The orientation and control values.
                '''
                control = 0
                searched = 0
                valor = back.pop()

                for valor in half:
                    control += 1
                    valor = back.pop()

                    if refObj == valor:
                        searched += 1
                        # Check if the object is found in 3 consecutive pixels
                        if searched >= 3:
                            break
                    else:
                        searched = 0

                    # Reset control and searched if reached the end of a line
                    if control == self.line:
                        control = 0
                        searched = 0

                return orientation, control

            control = 0
            orientation = None
            v = 0.02

            # Split the image into front and back halves
            front = image[self.half:]
            back = image[:self.half]
            
            if refObj in front:
                control = 0
                orientation = v
                
                # Search for the object in the front half
                orientation, control = search_in_half(front, refObj, orientation)

            if refObj in back: 
                control = 0
                orientation = -v if not orientation else 0
                
                # Search for the object in the back half
                orientation, control = search_in_half(back, refObj, orientation)

            if control == 0:
                return [-1, -1]  # No object found in the image
            elif control >= ((self.line/2) - 4) and control <= self.line/2:
                return [orientation, 0]  # Object centered in the image
            elif control <= self.line/2:
                return [orientation, -v]  # Object to the left in the image
            else:
                return [orientation, v]   # Object to the right in the image

    class TargetControl():
        ''' PARAMETERS:

        id (int): Get object handle vision sensor
        _clientID (int): Id of the client with api remote
        '''

        id         = None
        _clientID  = None
        
        def __init__(self, clientID):
            # Get the object handle for the quadricopter target
            self.id = vrep.simxGetObjectHandle(clientID, 'Quadricopter_target', vrep.simx_opmode_blocking)[1]
            self._clientID = clientID

            # Initiate image streaming for the vision sensor
            vrep.simxGetObjectPosition(self._clientID, self.id, -1, vrep.simx_opmode_streaming)
            time.sleep(0.5)

        def getPosition(self):
            '''
            Goal: Get the target position

            Return: Coordinate of the target x, y, z
            '''
            return vrep.simxGetObjectPosition(self._clientID, self.id, -1, vrep.simx_opmode_buffer)[1]

        def setPosition(self, x, y, z):
            vrep.simxSetObjectPosition(self._clientID, self.id, -1, [x, y, z], vrep.simx_opmode_oneshot)

    class SonarSensor():
        ''' PARAMETERS:

        id (int): Get object handle sonar  sensor
        _clientID (int): Id of the client with api remote
        '''

        id         = None
        _clientID  = None

        def __init__(self, clientID):
            self.id = vrep.simxGetObjectHandle(clientID, 'Proximity_sensor', vrep.simx_opmode_blocking)[1]
            self._clientID = clientID
            vrep.simxReadProximitySensor(self._clientID, self.id, vrep.simx_opmode_streaming)
            time.sleep(0.5)

        def getStateColision(self):
            '''
            Goal: Checks if the quadricopter will collide.

            Output: True if will collide and false if not.
            '''
            return vrep.simxReadProximitySensor(self._clientID, self.id, vrep.simx_opmode_buffer)[1]