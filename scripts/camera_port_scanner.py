import cv2
import numpy as np

def capture_image_from_port(port):
    cap = cv2.VideoCapture(port)
    if not cap.isOpened():
        print(f"Could not open port {port}")
        return None
    ret, frame = cap.read()
    cap.release()
    if not ret:
        print(f"Could not capture image from port {port}")
        return None
    return frame

def main():
    images = []
    max_height = 0
    total_width = 0

    # Capture images from ports
    for port in range(6):
        image = capture_image_from_port(port)
        if image is not None:
            images.append((port, image))
            max_height = max(max_height, image.shape[0])
            total_width += image.shape[1]

    if not images:
        print("No images captured.")
        return

    # Create a blank canvas to concatenate images
    combined_image = np.zeros((max_height, total_width, 3), dtype=np.uint8)

    # Concatenate images horizontally
    current_width = 0
    for port, image in images:
        h, w = image.shape[:2]
        combined_image[:h, current_width:current_width+w] = image
        cv2.putText(combined_image, f"Port {port}", (current_width + 20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
        current_width += w

    # Display concatenated image
    cv2.imshow("Combined Images", combined_image)
    cv2.waitKey(0)
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
