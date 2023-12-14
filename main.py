from drone.quadricopter import Quadricopter
from scene.scene_map import SceneMap

SPEED = 0.05
REF_OBJ = 23

if __name__ == "__main__":
    sMap = SceneMap(-8, 8, -8, 8, 0.5, 3)
    quadricopter = Quadricopter(24, vMin=SPEED)
    if quadricopter._clientID != -1:
        print("Server Connect!")
        quadricopter.startPosition(SPEED, sMap)
        
        # Esto no se hasta que punto esta bien
        while not isinstance(quadricopter._objFound, bool):
            quadricopter.searchObj(sMap)

        while quadricopter._objFound:
            if quadricopter.land(sMap):
                print(quadricopter.msg)
                break
            else:
                quadricopter.searchObj(sMap)
                print(quadricopter.msg)
        else:
            print(quadricopter.msg)
    else:
        print("Server offline!")