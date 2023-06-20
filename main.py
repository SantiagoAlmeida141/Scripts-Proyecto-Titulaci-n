#Importamos las librerias
import cv2
import numpy as np
from PIL import Image
from opcua import client

#Declaramos las variables
global cap
global opcion
global cont
global a

#Creamos las funciones
def conectarOPCUA():
    pass

def revisarSensor():
    pass

def enviarInstruccion():
    pass

def crearVideocaptura():
    global cap
    # Creamos la videocaptura
    cap = cv2.VideoCapture(0)
    pass

def capturarFrame():
    global cap
    # Leemos los fotogramas
    ret, frame = cap.read()

    # Conversiones
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    edg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    #print(ret)

    # Mostramos los frames
    cv2.imshow("VIDEOCAPTURA RGB", frame)
    cv2.imshow("VIDEOCAPTURA HSV", hsv)
    cv2.imshow("VIDEOCAPTURA EDG", edg)

    pass
def capturarFrameForma():
    global cap
    global cont

    #Se declara el contador sirve para indicar cuantos círculos (probetas) se han hallado
    cont=0

    #Leemos los fotogramas
    ret, frame = cap.read()

    #Conversiones
    edg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    mask = cv2.adaptiveThreshold(edg, 140, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 7)
    contours, hier = cv2.findContours(image=mask, mode= cv2.RETR_TREE, method= cv2.CHAIN_APPROX_NONE)
    cnt = cv2.drawContours(image=mask, contours= contours, contourIdx=-1, color=(0,255,0), thickness= 2)

    #Analizando cada contorno
    for cnt in contours:
        #Se filtra en base al valor del área
        #El rango del valor es experimental
        area = cv2.contourArea(cnt)
        #print(area)

        if (area<27000 and area>24000): #Rango para una determinada altura de la cámara

            (x,y),radio = cv2.minEnclosingCircle(cnt)
            centro = (int(x), int(y))
            radio = int(radio)
            circ = cv2.circle(frame, centro, radio, (0,255,0), 2)

            #Se cuenta el número de circulos encontrados
            cont = cont + 1

            cv2.imshow("CIRCULOS", circ)
            #El radio sirve para calcular el área que se debe filtrar
            #print(radio)

    #print(ret)

        # Mostramos los frames
        cv2.imshow("VIDEOCAPTURA RGB", frame)
        cv2.imshow("VIDEOCAPTURA EDG", edg)
        cv2.imshow("MÁSCARA", mask)
        #cv2.imshow("CONTORNOS", cnt)

    pass
def capturarFrameColor():
    global cap
    global cont

    # Color a buscar en BGR
    color = [0, 255, 255]

    # Leemos los fotogramas
    ret, frame = cap.read()

    # Conversiones
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    edg = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    maskcont = cv2.adaptiveThreshold(edg, 140, cv2.ADAPTIVE_THRESH_MEAN_C, cv2.THRESH_BINARY, 11, 7)
    contours, hier = cv2.findContours(image=maskcont, mode=cv2.RETR_TREE, method=cv2.CHAIN_APPROX_NONE)
    cnt = cv2.drawContours(image=maskcont, contours=contours, contourIdx=-1, color=(0, 255, 0), thickness=2)

    lowerLimit, upperLimit = get_limits(color=color)
    mask = cv2.inRange(hsv, lowerLimit, upperLimit)
    mask_ = Image.fromarray(mask)
    bbox = mask_.getbbox()
    #mask_ = Image.fromarray(mask)

    # Analizando cada contorno
    for cnt in contours:
        # Se filtra en base al valor del área
        # El rango del valor es experimental
        area = cv2.contourArea(cnt)
        # print(area)

        if (area < 20000 and area > 10000):  #Rango para una determinada altura de la cámara

            (x, y), radio = cv2.minEnclosingCircle(cnt)
            centro = (int(x), int(y))
            radio = int(radio)
            #Dibujando los círculos (probetas) encontraos
            circ = cv2.circle(frame, centro, radio, (0, 255, 0), 2)
            # El radio sirve para calcular el área que se debe filtrar
            #print(radio)

            #Mostrando el recuadro solo si se ha detectado el color ingresado
            #Revisar si bbox puede ser reemplazado por un círculo
            if bbox is not None:
                x1, y1, x2, y2 = bbox
                frame = cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
                cv2.imshow("COLOR DETECTADO", frame)

    #print(ret)

    # Mostramos los frames
    #cv2.imshow("VIDEOCAPTURA RGB", frame)
    cv2.imshow("VIDEOCAPTURA HSV", hsv)
    cv2.imshow("VIDEOCAPTURA EDG", edg)
    cv2.imshow("MÁSCARA", mask)

    pass
def get_limits(color):

    c = np.uint8([[color]]) # Insertando los valores BGR que se deesean pasar a HSV
    hsvC = cv2.cvtColor(c, cv2.COLOR_BGR2HSV)

    lowerLimit = hsvC[0][0][0] - 10, 100, 100
    upperLimit = hsvC[0][0][0] + 10, 255, 255

    lowerLimit = np.array(lowerLimit, dtype=np.uint8)
    upperLimit = np.array(upperLimit, dtype=np.uint8)

    return lowerLimit, upperLimit
def clasificarImagen():
    global opcion
    val = False
    while val == False:
        print("Elija (1) para clasificar por color o (2) para clasificar por defecto")
        opcion = input("Ingrese el valor: ")
        opcion = int(opcion)

        if opcion == 1:
            print("Se va a clasificar por color")
            val = True
            return True
        elif opcion == 2:
            print("Se va a clasificar por forma")
            val = False
            return True
        elif opcion !=1 or opcion !=2:
            print("El número ingresado no es válido")
            val = False
    pass
pass
def exitScript():
    # Se está saliendo del Script de forma manual presionando la tecla ESC
    # Cerramos con la lectura del teclado ESC(Código ASCII=27)
    t = cv2.waitKey(5)
    if t == 27:
       # Liberamos la videocaptura
       cap.release()
       # Cerramos las ventanas
       cv2.destroyAllWindows()
       #print("Probetas encontradas:", cont)
       print("Script cerrado")
    return False

if __name__ == '__main__':

    #conectarOPCUA()
    crearVideocaptura()

    if clasificarImagen() == True:
        while 1:
            if exitScript():
                break
            else:
                #El siguiente if utiliza una variable global
                if opcion == 1:
                    capturarFrameColor()
                elif opcion == 2:
                    capturarFrameForma()
            pass
        pass
    pass
