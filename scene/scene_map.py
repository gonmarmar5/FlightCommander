class SceneMap():
    ''' Parameters:

    xMin (float): Minimum coordinate that the map displays
    xMax (float): Maximum coordinate that the map displays
    yMin (float): Minimum coordinate that the map displays
    yMax (float): Maximum coordinate that the map displays
    zMin (float): Minimum coordinate that the map displays
    zMax (float): Maximum coordinate that the map displays
    '''

    xMin = None
    xMax = None
    yMin = None
    yMax = None
    zMin = None
    zMax = None

    def __init__(self, xMin, xMax, yMin, yMax, zMin, zMax):
        self.xMin = xMin
        self.xMax = xMax
        self.yMin = yMin
        self.yMax = yMax
        self.zMin = zMin
        self.zMax = zMax