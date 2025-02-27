import cv2

cap = cv2.VideoCapture(2)  # Cambia el índice según lo que encontraste


if not cap.isOpened():
    print("No se pudo abrir la cámara")
else:
    while True:
        ret, frame = cap.read()
        if not ret:
            print("No se pudo capturar el frame")
            break

        cv2.imshow("Astra Pro Plus", frame)

        # Presiona 'q' para salir
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()
