# https://www.quora.com/How-can-I-detect-a-rectangle-using-OpenCV-code-in-Python
# + chatGPT
import cv2
import numpy as np
from pyzbar.pyzbar import decode
import requests
import imutils


URL = "http://192.168.0.167:8080/shot.jpg"
COLOR_RANGES = {
    "GREEN" : {
        "lower": np.array([40, 40, 40]),
        "upper" : np.array([86,255, 255]),
    },

    "RED" : {
        "lower": np.array([0,100,100]),
        "upper": np.array([20, 255, 255]),
    },

    "BLUE" : {
        "lower": np.array([101,50,38]),
        "upper":np.array([110, 255, 255]),
    },

    "SKIN" : {
        "lower": np.array([0, 48, 80]),
        "upper": np.array([20, 255, 255]),
    }

}


def find_work_area(image):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

    # Check if qr code is present
    right_qr_code_found = False
    qr_codes = decode(gray)
    for code in qr_codes:
        qr_data = code.data.decode('utf-8')
        if qr_data == "Top right space":
            right_qr_code_found = True
            break

    if not right_qr_code_found:
        print("place not found")
        return # TODO error here

    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    edges = cv2.Canny(blurred, 50, 150)

    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    
    # Filter contours 
    rects = [] 
    for contour in contours: 
        # Approximate the contour to a polygon 
        polygon = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True) 

        # Check if the polygon has at least 3 sides 
        if len(polygon) == 4: # and abs(1 - cv2.contourArea(polygon) / (cv2.boundingRect(polygon)[2] * cv2.boundingRect(polygon)[3])) < 0.1: 
            rects.append(polygon) 
    
    return max(rects, key=cv2.contourArea)



    
def contains_color(image, contour, lower_boundary, upper_boundary):
    # mask image by contour
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.drawContours(mask, [contour], -1, 255, thickness=cv2.FILLED)
    
    # Apply the mask to the image
    masked_image = cv2.bitwise_and(image, image, mask=mask)
    hsv = cv2.cvtColor(masked_image, cv2.COLOR_BGR2HSV)
    # Mask by color
    mask = cv2.inRange(hsv, lower_boundary, upper_boundary)
   
    matching_pixels = cv2.countNonZero(mask)
    #cv2.imshow("Rectangles", masked_image)
    #if matching_pixels > 1000:
        #print(matching_pixels)
        #ims = cv2.resize(mask, (960, 540))  
        # 
    return matching_pixels > 1000

def main():
    while True:
        img_resp = requests.get(URL)
        img_arr = np.array(bytearray(img_resp.content), dtype=np.uint8) 
        img = cv2.imdecode(img_arr, -1)
        image = imutils.resize(img, width=1000, height=1800)  
        #cv2.imshow("Rectangles", image) 
        #image = cv2.imread('cardboard_image7.jpg')
        try:
            largest_rect = find_work_area(image)
            for color in COLOR_RANGES:
                color_included = contains_color(image, largest_rect, COLOR_RANGES[color]["lower"], COLOR_RANGES[color]["upper"])
                print(f"{color} included: {color_included}")
    
        except Exception as e:
            print("area not found")
   
        
        #cv2.drawContours(image, [largest_rect], 0, (0,255,0), 8)
        #cv2.imshow("Rectangles", image)
        # Crop the image to the rectangle
    
    ## Draw rectangles 
    #for rect in rects: 
    #    cv2.drawContours(image, [rect], 0, (0, 255, 0), 2) 
    
    # Show the result 
    
        #cv2.waitKey(0) 
        #cv2.destroyAllWindows() 


    


if __name__=="__main__":
    main()