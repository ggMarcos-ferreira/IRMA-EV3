#!/usr/bin/env pybricks-micropython
#!/usr/bin/env pybricks-micropython
from pybricks.hubs import EV3Brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import Port, Stop, Direction, Button, Color
from pybricks.tools import wait, StopWatch, DataLog
from pybricks.robotics import DriveBase
from pybricks.media.ev3dev import SoundFile, ImageFile

import socket
import sys
import os  
import os.path
import time

TKEYPRESS = 250

def detectMotoresSensores(ev3, falar):
    # Motores
    try:
        motorLeft = Motor(Port.D)
    except OSError:
        if falar:
            ev3.speaker.say("Motor esquerdo não encontrado.")
            ev3.speaker.say("Conecte o motor esquerdo na porta D")
        return
    try:
        motorRight = Motor(Port.A)
    except OSError:
        if falar:
            ev3.speaker.say("Motor direito não encontrado.")
            ev3.speaker.say("Conecte o motor direito na porta A")
        return
    if falar:
        ev3.speaker.say("Motores encontrados")
    
    # Sensores
    try:
        infrared = InfraredSensor(Port.S1)
    except OSError:
        if falar:
            ev3.speaker.say("Sensor infravermelho não encontrado.")
            ev3.speaker.say("Conecte o sensor infravermelho na porta S1")
        return
    try:
        corLeft = ColorSensor(Port.S3)
    except OSError:
        if falar:
            ev3.speaker.say("Sensor de cor esquerdo não encontrado.")
            ev3.speaker.say("Conecte o sensor de cor esquerdo na porta S3")
        return
    try: 
        corRight  = ColorSensor(Port.S2)
    except OSError:
        if falar:
            ev3.speaker.say("Sensor de cor direito não encontrado.")
            ev3.speaker.say("Conecte o sensor de cor direito na porta S2")
        return
    if falar:
        ev3.speaker.say("Sensores encontrados")
    
    return (motorLeft, motorRight, infrared, corLeft, corRight)

def initNetwork(ev3, falar):
    if(falar):
        ev3.speaker.say("Iniciando interface de rede")
    HOST = '192.168.1.133'    # Endereco IP do Servidor
    PORT = 2508             # Porta que o Servidor esta
    udp = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    dest = (HOST, PORT)

    return (udp, dest)

def detectaNovoEstado(ev3, cronometro, estado, MotorSensor, parametros):
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor
    (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR) = parametros

    novoEstado = estado
    botoes = ev3.buttons.pressed()

    if (Button.CENTER in botoes) and (estado == "PARADO") and (cronometro.time() >= TKEYPRESS):
        novoEstado = "ANDANDO"
        PoCorL = corLeft.rgb()[2]
        PoCorR = corRight.rgb()[2]
        cronometro.reset()
    elif (Button.CENTER in botoes) and (estado == "ANDANDO") and (cronometro.time() >= TKEYPRESS):
        novoEstado = "PARADO"
        cronometro.reset()
    elif (Button.RIGHT in botoes) and (estado != "DETECTAR_VERDE") and (cronometro.time() >= TKEYPRESS):
        novoEstado = "DETECTAR_VERDE"
        cronometro.reset()

    parametros = (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR)
    return (novoEstado, parametros)

def executaEstado(ev3, MotorSensor, estado, parametros):
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor
    (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR) = parametros

    CorL = corLeft.rgb()[2]
    CorR = corRight.rgb()[2]

    if estado == "ANDANDO":
        if CorL < 10 and CorR < 10:  # Encruzilhada com preto
            motorLeft.run(50)
            motorRight.run(50)
            wait(300)
        else:
            PotL = (PoCorL - CorL) * KpL + PoMotorL
            PotR = (PoCorR - CorR) * KpR + PoMotorR

            if -40 < PotL < 10:
                PotL = -40
            if -40 < PotR < 10:
                PotR = -40

            motorLeft.run(PotL)
            motorRight.run(PotR)

    elif estado == "PARADO":
        motorLeft.stop()
        motorRight.stop()

    elif estado == "DETECTAR_VERDE":
        motorLeft.stop()
        motorRight.stop()

        if CorL > 10:
            motorLeft.run(50)
            motorRight.run(50)
            wait(400)
            motorLeft.stop()
            motorRight.stop()
            wait(500)
            motorLeft.run_angle(200, 360)
            motorRight.run_angle(-200, 360)
            wait(500)
            motorLeft.run(50)
            motorRight.run(50)
            wait(300)

    elif estado == "GIRO_180_GRAUS":
        motorLeft.run_angle(200, 360)
        motorRight.run_angle(-200, 360)
        wait(500)

def enviaDados(conn, MotorSensor, contador):
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor
    (udp, dest) = conn

    left = corLeft.rgb()
    right = corRight.rgb()

    lr = "{0:.2f}".format(left[0])
    lg = "{0:.2f}".format(left[1])
    lb = "{0:.2f}".format(left[2])

    rr = "{0:.2f}".format(right[0])
    rg = "{0:.2f}".format(right[1])
    rb = "{0:.2f}".format(right[2])

    contador += 1

    msg = str(contador) + " " + lr + " " + lg + " " + lb + " " + rr + " " + rg +" " + rb

    udp.sendto(msg.encode(), dest)

    return contador

def main(argv):
    rede = False
    fala = not "-f" in argv
    rede = "-n" in argv

    ev3 = EV3Brick()
    ev3.speaker.set_speech_options('pt-br', 'm3')

    if rede:
        conn = initNetwork(ev3, fala)

    MotorSensor = detectMotoresSensores(ev3, fala)
    (motorLeft, motorRight, infrared, corLeft, corRight) = MotorSensor   

    cronometro = StopWatch()
    contador = 0
    PoCorL = 0
    KpL = -1.7
    PoMotorL = 60

    PoCorR = 0
    KpR = -1.2
    PoMotorR = 60

    parametros = (PoCorL, KpL, PoMotorL, PoCorR, KpR, PoMotorR)
    estado = "PARADO"
    ev3.speaker.beep()

    while True:
        if rede:
            contador = enviaDados(conn, MotorSensor, contador)
        
        NovoEstado, parametros = detectaNovoEstado(ev3, cronometro, estado, MotorSensor, parametros)
        
        executaEstado(ev3, MotorSensor, NovoEstado, parametros)
        
        estado = NovoEstado

main(sys.argv)

#O código não está completo, ajustes e manutenção com alguns valores precisam ser refeitos e testados. 
