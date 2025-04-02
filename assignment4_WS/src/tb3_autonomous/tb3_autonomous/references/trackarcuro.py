"""
Some portions of the code were generated using ChatGPT-4o
"""

import cv2
import numpy as np
import time
import os

def order_points(pts):
    """
    Orders 4 points in the following order:
    top-left, top-right, bottom-right, bottom-left.
    """
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]
    rect[2] = pts[np.argmax(s)]
    
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]
    rect[3] = pts[np.argmax(diff)]
    
    return rect

def load_templates():
    """
    Loads arrow templates in grayscale and returns them in a dictionary.
    """
    ar1 = cv2.imread('output0.jpg', 0)
    ar2 = cv2.imread('output1.jpg', 0)
    ar3 = cv2.imread('output2.jpg', 0)
    ar4 = cv2.imread('output3.jpg', 0)

    if ar1 is None or ar2 is None or ar3 is None  or ar4 is None:
        raise FileNotFoundError("Error: One or more arrow templates could not be loaded.")
    
    return {
        'arguco1': ar1,
        'arguco2': ar2,
        'arguco3': ar3,
        'arguco4': ar4
    }

def trackArrows_CV(frame, templates, threshold=0.6):
    """
    Processes the input image to detect arrow markers.
    It resizes and pre-processes the image, extracts candidate regions,
    performs perspective transformation, and applies template matching.
    
    Args:
        frame: Input frame.
        templates: A dictionary of arrow templates.
    
    Returns:
        annotated: the resized image with detection overlays.
        edges: the Canny edge image.
        detected_arrow: the label of the detected arrow (if any), else None.
        detected_arrow_val: the value of the detected arrow (if any), else None.
    """
    # Preprocessing: resize, grayscale, blur, and edge detection.
    frame_resized = cv2.resize(frame, (640, 480))
    gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)
    edges = cv2.Canny(blurred, 50, 150)
    
    # Find contours in the edge image.
    contours, _ = cv2.findContours(edges.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    detected_arrow = None
    detected_arrow_val = None

    # Loop through contours to find potential arrow regions.
    for cnt in contours:
        area = cv2.contourArea(cnt)
        if area < 500:
            continue

        peri = 0.02 * cv2.arcLength(cnt, True)
        approx = cv2.approxPolyDP(cnt, peri, True)
        
        # Look for quadrilateral contours.
        if len(approx) == 4:
            pts = approx.reshape(4, 2)
            rect = order_points(pts)
            
            # Compute width and height for perspective transform.
            (tl, tr, br, bl) = rect
            widthA = np.linalg.norm(br - bl)
            widthB = np.linalg.norm(tr - tl)
            maxWidth = max(int(widthA), int(widthB))
            
            heightA = np.linalg.norm(tr - br)
            heightB = np.linalg.norm(tl - bl)
            maxHeight = max(int(heightA), int(heightB))
            
            dst = np.array([
                [0, 0],
                [maxWidth - 1, 0],
                [maxWidth - 1, maxHeight - 1],
                [0, maxHeight - 1]
            ], dtype="float32")
            
            # Apply perspective transformation.
            M = cv2.getPerspectiveTransform(rect, dst)
            warped = cv2.warpPerspective(gray, M, (maxWidth, maxHeight))
            
            # Use template matching to identify the arrow.
            for arrow_label, template in templates.items():
                template_h, template_w = template.shape[:2]
                roi_resized = cv2.resize(warped, (template_w, template_h))
                
                res = cv2.matchTemplate(roi_resized, template, cv2.TM_CCOEFF_NORMED)
                _, max_val, _, _ = cv2.minMaxLoc(res)
                
                if max_val > threshold:
                    detected_arrow = arrow_label
                    detected_arrow_val = max_val
                    cv2.drawContours(frame_resized, [approx], -1, (0, 255, 0), 2)
                    cv2.putText(frame_resized, f"{arrow_label} ({max_val:.2f})", 
                                tuple(approx[0][0]), cv2.FONT_HERSHEY_SIMPLEX, 
                                0.8, (0, 0, 255), 2)
                    break
            
            if detected_arrow is not None:
                break

    return frame_resized, edges, detected_arrow, detected_arrow_val

def video_loop():
    """
    Captures video from the camera and processes each frame using trackArrows_CV(frame, templates).
    Displays the annotated frame along with the edge image and FPS.
    Press 'q' to quit.
    """
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("[ERROR] Could not open video source:")
        return

    # Load templates.
    templates = load_templates()
    prev_frame_time = time.time()
    
    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break
        
        # Process the frame to detect arrows.
        annotated, edges, arrow, arrow_val = trackArrows_CV(frame, templates, 0.6)
        
        # Calculate and overlay FPS.
        new_frame_time = time.time()
        fps = 1.0 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        cv2.putText(annotated, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        cv2.imshow("Camera Feed", annotated)
        cv2.imshow("Edges", edges)
        
        # Exit if 'q' is pressed.
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
            
    cap.release()
    cv2.destroyAllWindows()

def test():
    """
    Captures video from the camera and processes each frame using trackArrows_CV(frame, templates).
    Displays the annotated frame along with the edge image and FPS.
    Press 'q' to quit.
    """
    
    folder_path = "test_result_cv";
    for file in os.listdir(folder_path):
        file_path = os.path.join(folder_path, file)
        if os.path.isfile(file_path):
            os.remove(file_path)
            print(f"Delete: {file_path}")

    cap = cv2.VideoCapture("test.mov")
    if not cap.isOpened():
        print("[ERROR] Could not open video source:")
        return

    # Load templates.
    templates = load_templates()
    prev_frame_time = time.time()
    
    frame_count = 0
    save_path = None;
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        
        # Process the frame to detect arrows.
        annotated, edges, arrow, arrow_val = trackArrows_CV(frame, templates, 0.5)
        
        # Calculate and overlay FPS.
        new_frame_time = time.time()
        fps = 1.0 / (new_frame_time - prev_frame_time)
        prev_frame_time = new_frame_time
        cv2.putText(annotated, f"FPS: {fps:.2f}", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
        
        frame_count += 1
        if arrow != None:
            save_path = os.path.join(folder_path, f"frame_{frame_count:04d}_{arrow}_{arrow_val:.2f}.jpg")
            cv2.imwrite(save_path, annotated)
        else:
            save_path = os.path.join(folder_path, f"frame_{frame_count:04d}.jpg")
            cv2.imwrite(save_path, frame)

        print(f"Save: {save_path}")
            
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    video_loop()
    #test()
