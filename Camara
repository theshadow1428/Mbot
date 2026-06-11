import cv2
import numpy as np
import math
import serial
import time

# Resolucion de la camara
ancho = 640
alto = 480

# Centro de la imagen y linea de referencia
centro = ancho // 2
linea_y = 420

# Parametros de control proporcional
angulo_centro = 90
ganancia = 0.23

# Configuracion de comunicacion
puerto = "/dev/ttyUSB0"
baudios = 9600

# ROI (Region de interes)
roi = np.array([[(80, 200), (100, 480), (540, 480), (560, 200)]], dtype=np.int32)

# Inicializar camara
camara = cv2.VideoCapture(0)
camara.set(cv2.CAP_PROP_FRAME_WIDTH, ancho)
camara.set(cv2.CAP_PROP_FRAME_HEIGHT, alto)

# Inicializar comunicacion serial con Arduino Mega
puerto_serial = serial.Serial(puerto, baudios)
time.sleep(2)

# Ultimo angulo valido detectado
ultimo_angulo = angulo_centro

# Calcula donde cruza una linea en la altura linea_y
def calcular_x(x1, y1, x2, y2, y):
    if y2 == y1:
        return (x1 + x2) // 2

    pendiente = (x2 - x1) / (y2 - y1)
    return int(x1 + pendiente * (y - y1))

# Deteccion de lineas y generacion del angulo de conduccion
def detectar_linea_y_angulo(frame):

    # Conversion a escala de grises
    gris = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Deteccion de bordes mediante Canny
    bordes = cv2.Canny(gris, 50, 150)

    # Aplicar ROI para analizar unicamente la pista
    mascara = np.zeros_like(bordes)
    cv2.fillPoly(mascara, roi, 255)
    roi_filtrada = cv2.bitwise_and(bordes, mascara)

    # Deteccion de lineas mediante Transformada de Hough
    lineas = cv2.HoughLinesP(roi_filtrada, 1, np.pi / 180, 50, 50, 10)

    posiciones_x = []

    if lineas is not None:
        for linea in lineas:

            x1, y1, x2, y2 = linea[0]

            # Calcular orientacion de la linea detectada
            angulo_linea = math.degrees(math.atan2(y2 - y1, x2 - x1)) % 180

            # Filtrar lineas validas
            if 20 < angulo_linea < 160:
                posiciones_x.append(calcular_x(x1, y1, x2, y2, linea_y))

    # Calcular centro de la pista
    if len(posiciones_x) >= 2:
        centro_linea = (min(posiciones_x) + max(posiciones_x)) // 2

    elif len(posiciones_x) == 1:
        centro_linea = posiciones_x[0]

    else:
        return None, frame

    # Calcular error respecto al centro de la imagen
    error = centro_linea - centro

    # Control proporcional para generar el angulo de conduccion
    angulo_conduccion = int(max(0, min(180, angulo_centro + error * ganancia)))

    # Mostrar punto central detectado
    cv2.circle(frame, (centro_linea, linea_y), 6, (255, 0, 0), -1)

    return angulo_conduccion, frame


# Bucle principal del sistema
while True:

    lectura_correcta, frame = camara.read()

    if not lectura_correcta:
        break

    # Procesar imagen capturada
    angulo_conduccion, visual = detectar_linea_y_angulo(frame)

    # Mantener ultimo valor si se pierde la linea
    if angulo_conduccion is None:
        angulo_conduccion = ultimo_angulo
    else:
        ultimo_angulo = angulo_conduccion

    # Enviar angulo calculado al Arduino Mega
    puerto_serial.write(f"{ultimo_angulo}\n".encode())

    print("Angulo:", ultimo_angulo)

    # Mostrar procesamiento en tiempo real
    cv2.imshow("Vision", visual)

    # Presionar Enter para finalizar
    if cv2.waitKey(1) == 13:
        break

puerto_serial.close()
camara.release()
cv2.destroyAllWindows()
