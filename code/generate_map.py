import cv2
import numpy as np
import matplotlib.pyplot as plt

def generate_binary_map(image_path):
    # Cargamos la imagen
    image = cv2.imread(image_path)

    if image is None:
        print(f"Error: Unable to load image from '{image_path}'.")
        return

    # Convertimos la imagen de color a blanco y negro
    hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

    lower_white = np.array([0, 0, 200])
    upper_white = np.array([255, 30, 255])
    white_mask = cv2.inRange(hsv_image, lower_white, upper_white)
    white_walls = cv2.bitwise_and(image, image, mask=white_mask)
    gray = cv2.cvtColor(white_walls, cv2.COLOR_BGR2GRAY)
    _, binary_map = cv2.threshold(gray, 128, 255, cv2.THRESH_BINARY)

    # Printeamos la imagen
    cv2.imshow('Original Image', image)
    cv2.imshow('Binary Map (White Walls)', binary_map)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

    plt.imshow(binary_map, cmap='gray')
    plt.axis('off')

    # Guardamos la imagen
    #plt.savefig("D:/Image_2.png", format='png', bbox_inches='tight')
    #print(f"Binary map saved as {'D:/Image_2.png'}")

if __name__ == "__main__":
    image_path = 'D:/image_map.png'
    
    generate_binary_map(image_path)
