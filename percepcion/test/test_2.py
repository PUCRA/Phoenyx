import cv2
import numpy as np
import pytesseract  
import traceback
import os
os.environ["QT_QPA_PLATFORM"] = "xcb"

def obtener_recorte(frame, log_level=0):
    try:
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])
        lower_red_1 = np.array([0, 50, 50])
        upper_red_1 = np.array([10, 255, 255])
        lower_red_2 = np.array([170, 50, 50])
        upper_red_2 = np.array([180, 255, 255])
        
        mask_blue = cv2.inRange(hsv_image, lower_blue, upper_blue)
        mask_red_1 = cv2.inRange(hsv_image, lower_red_1, upper_red_1)
        mask_red_2 = cv2.inRange(hsv_image, lower_red_2, upper_red_2)
        mask_red = cv2.bitwise_or(mask_red_1, mask_red_2)
        combined_mask = cv2.bitwise_or(mask_blue, mask_red)
        
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        cleaned_mask = cv2.morphologyEx(combined_mask, cv2.MORPH_OPEN, kernel)
        
        contours, _ = cv2.findContours(cleaned_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        for contour in contours:
            epsilon = 0.02 * cv2.arcLength(contour, True)
            approx = cv2.approxPolyDP(contour, epsilon, True)
            if len(approx) == 4 and cv2.contourArea(approx) > 500:
                x, y, w, h = cv2.boundingRect(approx)
                aspect_ratio = float(w) / float(h)
                if 0.8 < aspect_ratio < 1.2:
                    return frame[y:y+h, x:x+w]
        return None
    except Exception as e:
        print(f"Error: {e}\n{traceback.format_exc()}")
        return None

def detectar_color_bgr(image):
    avg_b, avg_g, avg_r = np.mean(image[:, :, 0]), np.mean(image[:, :, 1]), np.mean(image[:, :, 2])
    if avg_b > avg_r and avg_b > avg_g:
        return "Azul"
    elif avg_r > avg_b and avg_r > avg_g:
        return "Rojo"
    return "Indefinido"

def obtener_num(image):
    try:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        _, thresh = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY_INV)
        processed = cv2.morphologyEx(thresh, cv2.MORPH_ERODE, np.ones((5, 5), np.uint8))
        config = '--oem 3 --psm 10 -c tessedit_char_whitelist=0123456789'
        number = pytesseract.image_to_string(processed, config=config).strip()
        return int(number) if number.isdigit() else -1
    except Exception:
        return -1

def decision_making(data_list):
    count_red, count_blue, count_none = 0, 0, 0
    numbers = []
    for entry in data_list:
        if entry[0] == "Rojo":
            count_red += 1
        elif entry[0] == "Azul":
            count_blue += 1
        else:
            count_none += 1
        if entry[1] != -1:
            numbers.append(entry[1])
    
    print(f"Red: {count_red}, Blue: {count_blue}, None: {count_none}, Numbers: {numbers}")
    if count_red > count_blue:
        print("Decision: Red Dominates")
    elif count_blue > count_red:
        print("Decision: Blue Dominates")
    else:
        print("Decision: No clear winner")

def main():
    cap = cv2.VideoCapture(2)
    data_list = []
    while True:
        ret, frame = cap.read()
        if not ret:
            break
        recorte = obtener_recorte(frame)
        if recorte is not None:
            color_detected = detectar_color_bgr(recorte)
            num_detected = obtener_num(recorte)
            data_list.append([color_detected, num_detected])
            cv2.imshow("Recorte", recorte)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
    cap.release()
    cv2.destroyAllWindows()
    decision_making(data_list)

if __name__ == "__main__":
    main()
