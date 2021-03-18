#!/usr/bin/env python2
# -*- coding: utf-8 -*-
#import com_serie
# import com_serie_b

import PID_B_original as PID_B
#import PID
import PWM_soft
import ttk
from Tkinter import *
#import serial
import time
import os
import datetime
import thread
import Tkinter
import matplotlib
import matplotlib.dates as mdates
import numpy as np
import RPi.GPIO as GPIO
import matplotlib.pyplot as pyplot
from matplotlib.figure import Figure
from datetime import datetime, timedelta
from ModuloSensado import ModuloSensado
# from matplotlib.backend_bases import key_press_handler
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2TkAgg

from tendo import singleton
import sys
import tkMessageBox
try:
    unique = singleton.SingleInstance()
except singleton.SingleInstanceException:
    print("Error: Ya hay otra instancia de BIOFlo3 en ejecución")
    sys.exit()

matplotlib.use('TkAgg')
import numpy as np

import threading
os.chdir("/home/pi/bioflo")
lock = threading.Lock()

modulo_sensado = ModuloSensado()
def fecha_hora_actual():
    return datetime.now().strftime("%d/%m/%Y %H:%M:%S")
    
def hora_actual():
    return datetime.now().strftime("%H:%M:%S")   
    
default_pid_originales = dict(T=(25, 360, 60),
                                ph=(10, 2, 0.5),
                                od=(7, 50, 1),
                                rpm=(0.4, 10, 0.3))
                                
default_pid_grande = dict(T=(150, 100, 60),
                                ph=(3000, 10000, 0.1),
                                od=(15, 400, 10),
                                rpm=(0.6, 9, 0.3))
default_pid = dict(T=(80, 300, 0),
                                ph=(3000, 10000, 0.1),
                                od=(15, 400, 10),
                                rpm=(0.3, 9, 0.3))


def sys_log(msg):
    archivo = open("sys_log.txt", "a")
    archivo.write("[%s]"%fecha_hora_actual() + hora_actual() + msg + "\n")

#################### PANTALLA PRINCIPAL pp
pp = Tk()
pp.geometry('1300x800')
pp.wm_title("SISTEMA DE CONTROL")
######## definicion variables -- valor instantaneo
VI_t = DoubleVar()
# VI_f=DoubleVar()
VI_ph = DoubleVar()
VI_od = DoubleVar()
VI_frec = DoubleVar()

date_entry = StringVar()
s = "200001010000"

#################### VARIABLES ON-OFF AUTO-MANUAL
label_state_f = "OFF"
label_color_f = "red"
label_state_auto_f = "AUTO"

label_state_t = "OFF"
label_color_t = "red"
label_state_auto_t = "AUTO"

label_state_ph = "OFF"
label_color_ph = "red"
label_state_auto_ph = "AUTO"

label_state_od = "OFF"
label_color_od = "red"
label_state_auto_od = "AUTO"

label_state_registro = "SIN REGISTRAR"
label_color_state_reg = "red"

state_t = 'disabled'
state_ph = 'disabled'
state_f = 'disabled'
state_od = 'disabled'

MODO_label_B1 = "OFF"
MODO_label_B2 = "OFF"
MODO_label_B3 = "OFF"
MODO_label_B4 = "OFF"
MODO_label_MOTOR = "OFF"
MODO_label_SOLENOIDE = "OFF"
MODO_label_CALEFACTOR = "OFF"

label_color_B1 = "tomato"
label_color_B2 = "tomato"
label_color_B3 = "tomato"
label_color_B4 = "tomato"
label_color_MOTOR = "tomato"
label_color_SOLENOIDE = "tomato"
label_color_CALEFACTOR = "tomato"

MODO_B1_label = StringVar()
MODO_B2_label = StringVar()
MODO_B3_label = StringVar()
MODO_B4_label = StringVar()
MODO_MOTOR_label = StringVar()
MODO_SOLENOIDE_label = StringVar()
MODO_CALEFACTOR_label = StringVar()

getting = False
##########deficicion valores de SP_PID and OP_PID
OP_PID_temp = DoubleVar()
OP_PID_od = DoubleVar()
OP_PID_f = DoubleVar()
OP_PID_ph = DoubleVar()
OP_PID_frec = DoubleVar()

SP_PID_ph = DoubleVar()
SP_PID_frec = DoubleVar()
SP_PID_temp = DoubleVar()
SP_PID_od = DoubleVar()
SP_PID_f = DoubleVar()

P_t = DoubleVar()
I_t = DoubleVar()
D_t = DoubleVar()

P_od = DoubleVar()
I_od = DoubleVar()
D_od = DoubleVar()

P_ph = DoubleVar()
I_ph = DoubleVar()
D_ph = DoubleVar()

P_frec = DoubleVar()
I_frec = DoubleVar()
D_frec = DoubleVar()

Caudal_B1 = DoubleVar()
Caudal_B2 = DoubleVar()
Caudal_B3 = DoubleVar()

Caudal_B1.set(0.00)
Caudal_B2.set(0.00)
Caudal_B3.set(0.00)

############# Inicialization Variables de comunicacion

list_data = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
list_dato = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
list_date = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
list_hora = ['a', 'b', 'c', 'd', 'e', 'f', 'g', '0', '0', '0', '0', '0', '0']
list_data_anterior = None
list_file = []

t_data = 0.0
ph_data = 0.0
od_data = 0.0
frec_data = 0

state_PWM_od = 0
state_PWM_ph = 0

MODO_config = 1
ban_grafica = 0
ban_dianuevo = 0
ban_dianuevo_file = 0
ban_TEST = 0
ban_registrando = 0

p_cal_th = DoubleVar()
p_cal_tl = DoubleVar()

p_cal_phh = DoubleVar()
p_cal_phl = DoubleVar()

p_cal_odh = DoubleVar()
p_cal_odl = DoubleVar()

p_cal_frech = DoubleVar()
p_cal_frecl = DoubleVar()

REF_th = DoubleVar()
REF_tl = DoubleVar()
REF_phh = DoubleVar()
REF_phl = DoubleVar()
REF_odh = DoubleVar()
REF_odl = DoubleVar()
REF_frecl = DoubleVar()
REF_frech = DoubleVar()

ano_date = StringVar()
mes_date = StringVar()
dia_date = StringVar()
hora_date = StringVar()
minuto_date = StringVar()
aux_minuto = StringVar()
aux_minuto_graph = StringVar()
auxx_minuto_graph = StringVar()
ano_date_edit = StringVar()
mes_date_edit = StringVar()
dia_date_edit = StringVar()
hora_date_edit = StringVar()
minuto_date_edit = StringVar()

ban_5min = 0
ban_15min = 0
ban_10min = 0
ban_30min = 0
ban_hora = 0

aux_minuto_date = 0

pendiente = StringVar()
ordenada = StringVar()
###############################

###############################

MODO_temp = StringVar()
MODO_ph = StringVar()
MODO_od = StringVar()
MODO_frec = StringVar()
MODO_reg = StringVar()
register_date = StringVar()
sensing_date = StringVar()

MODO_config_temp = DoubleVar()
MODO_config_ph = DoubleVar()
MODO_config_od = DoubleVar()
MODO_config_frec = DoubleVar()

Producto_ph_B1 = StringVar()
Producto_ph_B1.set("ACIDO")

Producto_ph_B2 = StringVar()
Producto_ph_B2.set("BASE")

Producto_ph_B3 = StringVar()
Producto_ph_B3.set("NUTRIENTE")

var = IntVar()  # variable de control de tiempo de muestro
var.set(10)  # por defecto 10 min

#############inicialization PID and PWM
#################### Inicializacion  PWM
GPIO.setmode(GPIO.BOARD)
GPIO.setwarnings(False)
sensing_failures = 0
PIN_IDLE = 13

GPIO.setup(PIN_IDLE, GPIO.IN)
GPIO.setup(18, GPIO.OUT)  # salida PWM TEMP
GPIO.setup(22, GPIO.OUT)  # salida PWM FREC
GPIO.setup(16, GPIO.OUT)  # reset rene
GPIO.setup(29, GPIO.OUT)  # solenoide
GPIO.setup(31, GPIO.OUT)  # bomba 1
GPIO.setup(33, GPIO.OUT)  # bomba 2
GPIO.setup(35, GPIO.OUT)  # bomba 3
GPIO.setup(37, GPIO.OUT)  # bomba 4

output_temp = GPIO.PWM(18, 0.2)  # pin = XX , Frec= XX
output_frec = GPIO.PWM(22, 4)
#output_ph = PWM_soft.PWM(33, 1, "DOBLE", 35, 31) MODIFICADO
output_ph = PWM_soft.PWM(pin=33, T=1, tipo="DOBLE", pin_aux=35,)

### PIN de encendido del motor
#Cambio
PIN_ENCENDIDO_MOTOR = 15
GPIO.setup(PIN_ENCENDIDO_MOTOR, GPIO.OUT)
GPIO.output(PIN_ENCENDIDO_MOTOR, True)
####
ban_output_bomba1 = 0
ban_output_bomba2 = 0
ban_output_bomba3 = 0
ban_output_bomba4 = 0
ban_motor = 0
ban_solenoide = 0
ban_calefactor = 0

Temp = PID_B.PID(tipo="TEMP")
Frec = PID_B.PID(Integrator_max=15000.0, Integrator_min=-15000.0)
Od = PID_B.PID(Integrator_max=15000.0, Integrator_min=-15000.0)
Ph = PID_B.PID(tipo="DOBLE", Integrator_max = 100, Integrator_min=-100)


P_t.set(default_pid["T"][0])
I_t.set(default_pid["T"][1])
D_t.set(default_pid["T"][2])

P_od.set(default_pid["od"][0])
I_od.set(default_pid["od"][1])
D_od.set(default_pid["od"][2])

P_ph.set(default_pid["ph"][0])
I_ph.set(default_pid["ph"][1])
D_ph.set(default_pid["ph"][2])

P_frec.set(default_pid["rpm"][0])
I_frec.set(default_pid["rpm"][1])
D_frec.set(default_pid["rpm"][2])

T = Temp.getT()
output_temp.start(0)
output_frec.start(0)
output_ph.SetPWM(0, 0, 0)  # duty =0,estado =0 (OFF) , signo =0 (+)
############################# VARIABLES PARA LAS GRAFICAS

array_graph_hora = []
array_graph_t = []
array_graph_ph = []
array_graph_frec = []
array_graph_od = []
array_graph_tiempo = []
firstime = 0
index = 0
ban_aux_first_time = 0
ban_cargar_vector = 0
conteo_get = DoubleVar()
conteo_get.set(0)

for i in range(200):
    array_graph_hora.append(" ")
    array_graph_t.append(0.0)
    array_graph_ph.append(0.0)
    array_graph_od.append(0.0)
    array_graph_frec.append(0.0)
    array_graph_tiempo.append(i)

figg = Figure(figsize=(3, 3))
aa = figg.add_subplot(111)
aa.set_title("Grafica General", fontsize=16)
aa.set_ylabel("VARIABLES", fontsize=14)
aa.set_xlabel("TIEMPO", fontsize=14)
aa.set_ylim(-10, 100)
bb = aa.twinx()
bb.set_ylabel("RPM", color='r')
bb.set_ylim(-10, 1300)
bb.tick_params('y', colors='r')


def reset_Controlador_Pic(msg=""):
    GPIO.output(16, True)  ## Enciendo
    time.sleep(2)  ## Esperamos
    GPIO.output(16, False)
    sys_log("Reset: " + msg)
    

"""
def check_sensed_values(data, VI, minimum, maximum):
    if float(VI.get()) == 0:
        return True
    if data <= maximum and data >= minimum:
        #return abs(data - float(VI.get())) <= (0.8 * float(VI.get()))
        return True
        ########### Cambiar
    else:
        return False


def check_all_sensed_values():
    checks = []
    checks.append(check_sensed_values(t_data, VI_t, 0, 99))
    checks.append(True)#check_sensed_values(ph_data, VI_ph, 0, 15))
    checks.append(True) #check_sensed_values(od_data, VI_od, 0, 10))
    checks.append(check_sensed_values(frec_data, VI_frec, 0, 9999))
    return checks

"""
def log_data(list_data, pid_data):
    file_name_ = "log/%s.csv"%list_data.datetime.date()
    data = "%s, %s, %s, %s, %s, %s, %s, %s, %s, %s\n"%(list_data.datetime.date(),
                                                                                          list_data.datetime.time(),
                                                                                          list_data.T,
                                                                                          list_data.ph,
                                                                                          list_data.od,
                                                                                          list_data.rpm,
                                                                                          pid_data["T"],
                                                                                          pid_data["ph"],
                                                                                          pid_data["od"],
                                                                                          pid_data["rpm"])
    if not os.path.isfile(file_name_):
       data = "Fecha, Hora, T, pH, OD, rpm, PID_T, PID_pH, PID_OD, PID_rpm\n" + data
    data_log = open(file_name_, 'a')
    data_log.write(data)
    data_log.close()
last_measure = 0
############## call funcion COM_SERIE
file_name = str(datetime.now().date())
def get_data(port=None):
    global getting
    global hora_data
    global minute_data
    global t_data
    global ph_data
    global od_data
    global frec_data
    global MODO_config
    global date_pp
    global list_date
    global list_dato
    global list_hora
    global ano_date
    global mes_date
    global dia_date
    global hora_date
    global minuto_date
    global p_cal_th
    global p_cal_phh
    global p_cal_odh
    global p_cal_frech
    global p_cal_tl
    global p_cal_phl
    global p_cal_odl
    global p_cal_frecl
    global ban_dianuevo_file
    global ban_dianuevo
    global ano_date_edit
    global mes_date_edit
    global dia_date_edit
    global hora_date_edit
    global minuto_date_edit
    global array_graph_hora
    global array_graph_t
    global array_graph_ph
    global array_graph_od
    global array_graph_frec
    global index
    global ban_cargar_vector
    global array_graph_tiempo
    global firstime
    global auxx_minuto_graph
    global conteo_get
    global sensing_failures
    global list_data_anterior
    global last_measure
    global list_data
    global file_name
    dt = round(time.time() - last_measure, 1)
    if dt < 1:
        return 5
    
    if getting == True:
        sys_log("Saliendo sin hacer nada")
        return 2
    getting = False
    if MODO_config == 0:
        try:
            list_data = False
            with lock:
                list_data = modulo_sensado.get_data()
            if not list_data:
               sys_log("No se recibieron datos de medición")
               return 5
            else:
                
                last_measure = time.time()
                t_data = list_data.T
                ph_data = list_data.ph
                od_data = list_data.od
                frec_data = list_data.rpm
                fechayhora = list_data.datetime
                list_hora =[]
                list_hora.extend(("%.2d"%fechayhora.hour))
                list_hora.extend(("%.2d"%fechayhora.minute))
                list_data_anterior = list_data
                hora_date.set(str(list_hora[0]) + str(list_hora[1]))
                minuto_date.set(str(list_hora[2]) + str(list_hora[3]))
                VI_frec.set(frec_data)
                VI_t.set(t_data)
                VI_ph.set(ph_data)
                VI_od.set(od_data)
                conteo_get.set(conteo_get.get() + 1.0)
                
                if dt < 1:
                    return 5    
                if int(conteo_get.get()) % 5 == 0:  # antes en raspberry 2 estaba en 10 ahora en 5
                    for i in range(199):
                        array_graph_hora[i] = array_graph_hora[i + 1]
                        array_graph_t[i] = array_graph_t[i + 1]
                        array_graph_ph[i] = array_graph_ph[i + 1]
                        array_graph_od[i] = array_graph_od[i + 1]
                        array_graph_frec[i] = array_graph_frec[i + 1]
                        array_graph_tiempo[i] = i
                    array_graph_t[199] = t_data
                    array_graph_ph[199] = ph_data
                    array_graph_od[199] = od_data
                    array_graph_frec[199] = frec_data
                    array_graph_tiempo[199] = 200
                    if firstime == 0:
                        firstime = 1
                        print "first time"
                        array_graph_hora[199] = str(list_hora[0]) + str(list_hora[1]) + ":" + str(list_hora[2]) + str(
                            list_hora[3])
                        auxx_minuto_graph.set(minuto_date.get())
                    else:
                        if auxx_minuto_graph.get() != minuto_date.get():
                            try:
                                if int(minuto_date.get()) % 5 == 0:
                                    array_graph_hora[199] = str(list_hora[0]) + str(list_hora[1]) + ":" + str(
                                        list_hora[2]) + str(list_hora[3])
                                    auxx_minuto_graph.set(minuto_date.get())
                                else:
                                    array_graph_hora[199] = " "
                            except ValueError:
                                print "error"
                                sys_log("Error graficando en get_data")
                        else:
                            array_graph_hora[199] = " "

                    #if (str(list_hora[0]) + str(list_hora[1]) + ":" + str(list_hora[2]) + str(list_hora[3])) == "00:00": #cambiado
                    if file_name != str(list_data.datetime.date()):
                        if ban_dianuevo_file == 0:
                            # No es necesario
                            #MODO_config = 1
                            #ban_dianuevo = 1
                            ban_dianuevo_file = 1
                            #reset_Controlador_Pic("Reset nuevo dia")
                            print "dia nuevo"
                            sys_log("Día Nuevo")
                        else:
                            MODO_config = 0
                    else:
                        #date = str(list_date[0]) + str(list_date[1]) + "/" + str(list_date[2]) + str(
                        #    list_date[3]) + "/" + str(20) + str(list_date[4]) + str(list_date[5]) + " - " + str(
                        #    list_hora[0]) + str(list_hora[1]) + ":" + str(list_hora[2]) + str(list_hora[3])
                        variable_date.config(text=fecha_hora_actual())
                refreshPID(1)
                refreshPWM_soft_ph(1)
                getting = False
        except:
            print "error_com_inicial en mod_config_0"
            sys_log("error_con_inicial en mod_config_0")
            reset_Controlador_Pic("error_com_inicial get_data")
            #time.sleep(0.5)
            MODO_config = 0
            getting = False

    if MODO_config==1:
            with lock:
                list_date=modulo_sensado.get_datetime()     
            try:
                dia_date.set("%.02d"%list_date.day)
                mes_date.set("%.02d"%list_date.month)
                ano_date.set(str(list_date.year))
                hora_date.set("%.02d"%list_date.hour)
                minuto_date.set("%.02d"%list_date.minute)
                print "anio:",ano_date.get()
                print "mes:",mes_date.get()
                print "dia:",dia_date.get()
                print "hora:",hora_date.get()
                print "minuto:",minuto_date.get()       
                MODO_config=0
                date =str(list_date)
                variable_date.config(text=date)
                system_date = os.system("sudo date "+mes_date.get()+dia_date.get()+hora_date.get()+minuto_date.get()+str(list_date.year-2000))
                msg = "Fecha configurada: %s/%s/%s %s:%s"%(dia_date.get(), mes_date.get(), list_date.year, hora_date.get(),minuto_date.get())
            	sys_log(msg)
            	tkMessageBox.showinfo("", "Verificar Fecha y Hora. \n"+ msg)
            	# fix error fecha pasada detiene el proceso de control
            	last_measure = time.time()
            	
            	
            	
                print "Fecha configurada"
            except OSError:  
                print "error"
                sys_log("Error configurando fecha: %s"%modulo_sensado.get_datetime())
    if MODO_config==2:
        with lock:
            modulo_sensado.set_datetime(ano_date_edit.get(),
                                                                    mes_date_edit.get(),
                                                                    dia_date_edit.get(),
                                                                    hora_date_edit.get(),
                                                                    minuto_date_edit.get()
                                                                    )
            MODO_config = 1

    if MODO_config == 3:  # calibracion Temp
        with lock:
            list_date = modulo_sensado.get_slope("T")
        p_cal_th.set(list_date[0])
        print "pendiente inicial - T: ", p_cal_th.get()
        p_cal_tl.set(list_date[1])
        print "ordenada inicial - T: ", p_cal_tl.get()
        MODO_config = 0

    if MODO_config == 4:  # calibracion PH
        with lock:
            list_date = modulo_sensado.get_slope("ph")
        p_cal_phh.set(list_date[0])
        print "pendiente inicial - ph: ", p_cal_phh.get()
        p_cal_phl.set(list_date[1])
        print "ordenada inicial - ph: ", p_cal_phl.get()
        MODO_config = 0
            
    if MODO_config == 5:  # calibracion OD
        with lock:
            list_date = modulo_sensado.get_slope("od")
        p_cal_odh.set(list_date[0])
        print "pendiente inicial - od: ", p_cal_odh.get()
        p_cal_odl.set(list_date[1])
        print "ordenada inicial - od: ", p_cal_odl.get()
        MODO_config = 0
        
    if MODO_config == 6:  # calibracion RPM
        with lock:
            list_date = modulo_sensado.get_slope("rpm")
        p_cal_frech.set(list_date[0])
        print "pendiente inicial - rpm: ", p_cal_frech.get()
        p_cal_frecl.set(list_date[1])
        print "ordenada inicial - rpm: ", p_cal_frecl.get()
        MODO_config = 0
        
    if MODO_config == 7:  # RESET VALORES
            print "reseteando las pendientes y constantes"
            time.sleep(2)
            
    if MODO_config == 8:  # espera terminar de graficar
        time.sleep(1)
    getting = False


#############################REFRESH DATA
def refreshfile(cont):
    time.sleep(3)
    while (2):
        global hora_data
        global minute_data
        global t_data
        global ph_data
        global od_data
        global frec_data
        global date_pp
        global list_date
        global list_dato
        global list_hora
        global ano_date
        global mes_date
        global dia_date
        global hora_date
        global minuto_date
        global ban_dianuevo_file
        global aux_minuto
        global var
        global aux_minuto_graph
        global array_graph_hora
        global array_graph_t
        global array_graph_ph
        global array_graph_od
        global array_graph_frec
        global MODO_config
        global aa
        global figg
        global array_graph_tiempo
        global ban_grafica
        global ban_5min
        global ban_15min
        global ban_10min
        global ban_30min
        global ban_hora
        global list_file
        global ban_aux_first_time
        global file_name

        time.sleep(3)

        if ban_grafica == 1:
            #if file_name != str(datetime.now.date())
            if ban_dianuevo_file == 1:
            
                ban_dianuevo_file = 0
                #file_name = str(dia_date.get()) + "-" + str(mes_date.get()) + "-" + str(ano_date.get())
                file_name = str(datetime.now().date())
                archivo = open("registro/"+file_name + ".csv", 'a')
                # agregado
                archivo2 = open("PID/" + file_name + ".csv", 'a')
                print "nuevo archivo"
                archivo.close()
                archivo2.close()
                for file in os.listdir("registro"):
                    if file.endswith(".csv"):
                        list_file.append(file)
                        combo["values"] = list_file
            if var.get() == 1:
                
                try:
                    if aux_minuto.get() != minuto_date.get():
                        print "%s - %s"%(aux_minuto.get(), minuto_date.get())
                        archivo = open("registro/"+file_name + ".csv", 'a')
                        archivo2 = open("PID/" + file_name + ".csv", 'a')

                        archivo.write(
                            str(hora_date.get()) + ":" + str(minuto_date.get()) + " , " + str(VI_t.get()) + ' , ' + str(
                                VI_ph.get()) + ' , ' + str(VI_od.get()) + ' , ' + str(VI_frec.get()) + '\n')
                        # agregado
                        archivo2.write(str(hora_date.get()) + ":" + str(minuto_date.get()) + " , " + str(
                            OP_PID_temperatura.get()) + " , " + str(OP_PID_phh.get()) + " , " + str(
                            OP_PID_odd.get()) + " , " + str(OP_PID_frecuencia.get() + "\n"))
                        aux_minuto.set(minuto_date.get())
                        print "escribiendo en archivo - 1 min"
                        archivo.close()
                        archivo2.close()
                except:
                    pass

            if var.get() == 5:
                try:
                    if int(minuto_date.get()) % 5 == 0:
                        if ban_5min == 0:
                            archivo = open("registro/"+file_name + ".csv", 'a')
                            archivo.write(str(hora_date.get()) + ":" + str(minuto_date.get()) + " , " + str(
                                VI_t.get()) + ' , ' + str(VI_ph.get()) + ' , ' + str(VI_od.get()) + ' , ' + str(
                                VI_frec.get()) + '\n')
                            print "escribiendo en archivo - 5 min"
                            aux_minuto = 1
                            ban_5min = 1
                            archivo.close()
                    else:
                        ban_5min = 0
                except ValueError:
                    pass
            if var.get() == 10:
                try:
                    if int(minuto_date.get()) % 10 == 0:
                        if ban_10min == 0:
                            archivo = open("registro/"+file_name + ".csv", 'a')
                            archivo.write(str(hora_date.get()) + ":" + str(minuto_date.get()) + " , " + str(
                                VI_t.get()) + ' , ' + str(VI_ph.get()) + ' , ' + str(VI_od.get()) + ' , ' + str(
                                VI_frec.get()) + '\n')
                            aux_minuto.set(minuto_date.get())
                            print "escribiendo en archivo - 10min"
                            archivo.close()
                            ban_10min = 1
                    else:
                        ban_10min = 0
                except ValueError:
                    pass

            if var.get() == 15:
                try:
                    if int(minuto_date.get()) % 15 == 0:
                        if ban_15min == 0:
                            archivo = open("registro/"+file_name + ".csv", 'a')
                            archivo.write(str(hora_date.get()) + ":" + str(minuto_date.get()) + " , " + str(
                                VI_t.get()) + ' , ' + str(VI_ph.get()) + ' , ' + str(VI_od.get()) + ' , ' + str(
                                VI_frec.get()) + '\n')
                            aux_minuto.set(minuto_date.get())
                            print "escribiendo en archivo - 15min"
                            archivo.close()
                            ban_15min = 1
                    else:
                        ban_15min = 0
                except ValueError:
                    pass
            if var.get() == 30:
                try:
                    if int(minuto_date.get()) % 30 == 0:
                        if ban_30min == 0:
                            archivo = open("registro/"+file_name + ".csv", 'a')
                            archivo.write(str(hora_date.get()) + ":" + str(minuto_date.get()) + " , " + str(
                                VI_t.get()) + ' , ' + str(VI_ph.get()) + ' , ' + str(VI_od.get()) + ' , ' + str(
                                VI_frec.get()) + '\n')
                            aux_minuto.set(minuto_date.get())
                            print "escribiendo en archivo - 30 min"
                            archivo.close()
                            ban_30min = 1
                    else:
                        ban_30min = 0
                except ValueError:
                    pass
            if var.get() == 60:
                print "minuto:", minuto_date.get()
                try:
                    if int(minuto_date.get()) == 0:
                        if ban_hora == 0:
                            archivo = open("registro/"+file_name + ".csv", 'a')
                            archivo.write(str(hora_date.get()) + ":" + str(minuto_date.get()) + " , " + str(
                                VI_t.get()) + ' , ' + str(VI_ph.get()) + ' , ' + str(VI_od.get()) + ' , ' + str(
                                VI_frec.get()) + '\n')
                            print "escribiendo en archivo - 60min"
                            archivo.close()
                            ban_hora = 1
                    else:
                        ban_hora = 0
                except ValueError:
                    pass
        if ban_aux_first_time == 0:
            time.sleep(1)
            ban_aux_first_time = 1
            print "descansando antes de graficar!"
        else:
            try:
                if aux_minuto_graph.get() != minuto_date.get():
                #if True:
                    MODO_config == 8
                    aux_minuto_graph.set(minuto_date.get())
                    aa.clear()
                    bb.clear()
                    bb.set_ylabel("RPM", color='r')
                    bb.set_ylim(-10, 1300)

                    aa.set_ylim(-10, 100)
                    aa.set_ylabel("TEMP - PH - OD", fontsize=14)

                    aa.set_xticks(array_graph_tiempo)
                    aa.set_xticklabels(array_graph_hora)
                    lines = []
                    lines += aa.plot(array_graph_tiempo, array_graph_t, label="TEMP", marker="o", linewidth=1,
                                     markersize=2)
                    lines += aa.plot(array_graph_tiempo, array_graph_ph, label="PH", marker="o", linewidth=1,
                                     markersize=2)
                    lines += aa.plot(array_graph_tiempo, array_graph_od, label="OD", marker="o", linewidth=1,
                                     markersize=2)
                    lines += bb.plot(array_graph_tiempo, array_graph_frec, 'r', marker="o", linewidth=1, markersize=2)
                    figg.legend(lines, ["TEMP", "PH", "OD", "RPM"], loc=2)
                    figg.canvas.draw()
                    MODO_config = 0
            except AttributeError, ValueError:
                print("Excepción al graficar")
                sys_log("Exepción al graficar en refresh file")
                pass


def refreshData(cont):
    while (1):
        global MODO_config
        if MODO_config == 0:
            get_data()
            time.sleep(1)
        else:
            if MODO_config == 1:
                get_data()
            else:
                get_data()


def refreshPWM_soft_ph(cont):
    global state_PWM_ph
    if state_PWM_ph == 1:
        try:
            if int(OP_PID_ph.get()) < 0:
                output_ph.SetPWM(int(OP_PID_ph.get()) * (-1), state_PWM_ph, 1)
            else:
                output_ph.SetPWM(int(OP_PID_ph.get()), state_PWM_ph, 0)
        except ValueError:
            print("Value error en refresh PWM_soft_ph")
    else:
        pass


def refreshPID(cont):
    global t_data
    global ph_data
    global od_data
    global frec_data
    global state_PWM_od
    global state_PWM_ph
    global ban_TEST
    
    ##TEMPERATURA
    if label_state_t == "ON":
        if label_state_auto_t == "AUTO":
            try:
                Temp.setKp(P_t.get())
                Temp.setKi(I_t.get())
                Temp.setKd(D_t.get())
                Temp.setPoint(SP_PID_temp.get())
                PID = Temp.update(t_data)
                if PID:
                    OP_PID_temp.set(round(PID, 1))
                    if PID == -1:
                        GPIO.output(29, True)  ## Enciendo
                        OP_PID_temp.set(0)
                    else:
                        GPIO.output(29, False)  ## Apagado
                        #Temp.setIntegrator(Temp.getIntegrator())
                        output_temp.ChangeDutyCycle(int(OP_PID_temp.get()))
            except ValueError:
                pass
        else:
            try:
                output_temp.ChangeDutyCycle(int(OP_PID_temp.get()))
            except ValueError:
                pass
    else:
        if ban_TEST == 0:
            output_temp.ChangeDutyCycle(0)
            OP_PID_temp.set(0.0)

    ##### OD #######

    if label_state_od == "CASCADA":
        try:
            Od.setKp(P_od.get())
            Od.setKi(I_od.get())
            Od.setKd(D_od.get())
            Od.setPoint(SP_PID_od.get())
            OP_PID_od.set(round(Od.update(od_data), 1))
            Od.setIntegrator(Od.getIntegrator())
            if (SP_PID_frec.get() - frec_data) > 0 and Od.getError() < 0:  # Se cambio or por and
                print("Gobierna PID RPM: (%s, %s)"%(SP_PID_frec.get() - frec_data, Od.getError()))
                try:
                    Frec.setKp(P_frec.get())
                    Frec.setKi(I_frec.get())
                    Frec.setKd(D_frec.get())
                    Frec.setPoint(SP_PID_frec.get())
                    OP_PID_frec.set(round(Frec.update(frec_data), 1))
                    Frec.setIntegrator(Frec.getIntegrator())
                    output_frec.ChangeDutyCycle(int(OP_PID_frec.get()))
                except ValueError:
                    pass
            else:
                sum_ = min(int(OP_PID_od.get()) + int(OP_PID_frec.get()), 100)
                output_frec.ChangeDutyCycle(sum_)
                
                print "TRABAJANDO CON LA SUMA ", int(OP_PID_od.get()),  int(OP_PID_frec.get()), sum_


        except ValueError:
            pass
    else:
        state_PWM_od = 0
        OP_PID_od.set(0.0)

        ##### PH  #######
    if label_state_ph == "ON":
        state_PWM_ph = 1
        if label_state_auto_ph == "AUTO":
            try:
                Ph.setKp(P_ph.get())
                Ph.setKi(I_ph.get())
                Ph.setKd(D_ph.get())
                Ph.setPoint(SP_PID_ph.get())
                OP_PID_ph.set(round(Ph.update(ph_data), 1))
                Ph.setIntegrator(Ph.getIntegrator())
            except ValueError:
                pass
    else:
        state_PWM_ph = 0
        OP_PID_ph.set(0.0)

    ##### FRECUENCIA  #######
    if label_state_f == "ON":
        if label_state_auto_f == "AUTO":
            try:
                Frec.setKp(P_frec.get())
                Frec.setKi(I_frec.get())
                Frec.setKd(D_frec.get())
                Frec.setPoint(SP_PID_frec.get())
                output_PID = round(Frec.update(frec_data), 1)
                OP_PID_frec.set(round(Frec.real_PID, 1))
                # Frec.setIntegrator(Frec.getIntegrator())
                output_frec.ChangeDutyCycle(output_PID)
            except ValueError:
                pass
        else:
            try:
                output_frec.ChangeDutyCycle(int(OP_PID_frec.get()))
            except ValueError:
                pass
    else:
        if label_state_od != "CASCADA":
            if ban_TEST == 0:
                output_frec.ChangeDutyCycle(0)
                OP_PID_frec.set(0.0)
    pid_data = dict(T = OP_PID_temp.get(), ph=OP_PID_ph.get(), od=OP_PID_od.get(), rpm=OP_PID_frec.get())
    log_data(list_data, pid_data)





###################### P principal con todos los frames


notebook_style = ttk.Style()
notebook_style.configure("TNotebook", font=("calibri", 18))
notebook_style.configure("TNotebook.Tab", font=("calibri", 18))
notebook_style.configure("TCombobox", font=("calibri", 18))

notebook = ttk.Notebook(pp)
notebook.pack(fill='both', expand='yes', ipadx=12)
frame1 = ttk.Frame(notebook)
frame2 = ttk.Frame(notebook, width=80)
frame3 = ttk.Frame(notebook, width=100)
frame4 = ttk.Frame(notebook, width=120)
frame5 = ttk.Frame(notebook, width=140)
frame6 = ttk.Frame(notebook, width=180)

notebook.add(frame1, text='PRINCIPAL')
notebook.add(frame2, text='Calibración')
notebook.add(frame4, text='Bombas')
notebook.add(frame5, text='Configuración')
notebook.add(frame6, text='Historial')

date_pp = datetime(year=int(s[0:4]), month=int(s[4:6]), day=int(s[6:8]), hour=int(s[8:10]), minute=int(s[10:12]))
variable_date = Label(pp, text=date_pp, font=("calibri", 12))
variable_date.pack()

########
##################################   FRAME 1 - VARIABLES Y VALORES ACTUALES ################################

variable = Label(frame1, text="Variables", font=("calibri", 18), width=14)
variable.pack()
variable.place(x=0, y=10)

valorI = Label(frame1, text="Valor Instantaneo", font=("calibri", 18), width=14)
valorI.pack()
valorI.place(x=250, y=10)

modo = Label(frame1, text=" Modo", font=("calibri", 18), width=14)
modo.pack()
modo.place(x=500, y=10)

SP_Label = Label(frame1, text="  SP", font=("calibri", 18), width=14)
SP_Label.pack()
SP_Label.place(x=750, y=10)

OP_Label = Label(frame1, text="Salida-PID", font=("calibri", 18), width=14)
OP_Label.pack()
OP_Label.place(x=1000, y=10)

# valores instant

VI_temperatura = Entry(frame1, textvariable=VI_t, justify=CENTER, font=("calibri", 18), width=15)
VI_temperatura.pack(ipady=3)
VI_temperatura.place(x=250, y=50)

VI_phh = Entry(frame1, textvariable=VI_ph, justify=CENTER, font=("calibri", 18), width=15)
VI_phh.pack(ipady=6)
VI_phh.place(x=250, y=100)

VI_odd = Entry(frame1, textvariable=VI_od, justify=CENTER, font=("calibri", 18), width=15)
VI_odd.pack(ipady=9)
VI_odd.place(x=250, y=150)

VI_frecuencia = Entry(frame1, textvariable=VI_frec, justify=CENTER, font=("calibri", 18), width=15)
VI_frecuencia.pack(ipady=12)
VI_frecuencia.place(x=250, y=200)

########modo


MODO_temperatura = Entry(frame1, textvariable=MODO_temp, bg=label_color_t, font=("calibri", 18), width=15,
                         justify=CENTER)
MODO_temperatura.pack()
MODO_temperatura.place(x=500, y=50)
MODO_temp.set(label_state_t)

MODO_phh = Entry(frame1, textvariable=MODO_ph, font=("calibri", 18), bg=label_color_ph, width=15, justify=CENTER)
MODO_phh.pack()
MODO_phh.place(x=500, y=100)
MODO_ph.set(label_state_ph)

MODO_odd = Entry(frame1, textvariable=MODO_od, justify=CENTER, bg=label_color_od, font=("calibri", 18), width=15)
MODO_odd.pack()
MODO_odd.place(x=500, y=150)
MODO_od.set(label_state_od)

MODO_frecuencia = Entry(frame1, textvariable=MODO_frec, bg=label_color_f, justify=CENTER, font=("calibri", 18),
                        width=15)
MODO_frecuencia.pack()
MODO_frecuencia.place(x=500, y=200)
MODO_frec.set(label_state_f)

MODO_registro = Entry(frame1, textvariable=MODO_reg, bg=label_color_state_reg, justify=CENTER, font=("calibri", 18),
                      width=15)
MODO_registro.pack()
MODO_registro.place(x=250, y=250)
MODO_reg.set(label_state_registro)

"""
entry_register_date = Entry(frame1, textvariable=register_date, bg=label_color_state_reg, justify=CENTER, font=("calibri", 18),
                      width=15)
entry_register_date.pack()
entry_register_date.place(x=250, y=250)
entry_register_date.set(label_state_registro)
"""
########## VALORES -- SP
# variables tipo float

SP_PID_temperatura = Entry(frame1, textvariable=SP_PID_temp, justify=CENTER, font=("calibri", 18), width=15)
SP_PID_temperatura.pack()
SP_PID_temperatura.place(x=750, y=50)

SP_PID_phh = Entry(frame1, textvariable=SP_PID_ph, justify=CENTER, font=("calibri", 18), width=15)
SP_PID_phh.pack()
SP_PID_phh.place(x=750, y=100)

SP_PID_odd = Entry(frame1, textvariable=SP_PID_od, justify=CENTER, font=("calibri", 18), width=15)
SP_PID_odd.pack()
SP_PID_odd.place(x=750, y=150)

SP_PID_frecuencia = Entry(frame1, textvariable=SP_PID_frec, justify=CENTER, font=("calibri", 18), width=15)
SP_PID_frecuencia.pack()
SP_PID_frecuencia.place(x=750, y=200)

########## SALIDA _ PID
# output-pid
# variables tipo float (op=out put pid)



OP_PID_temperatura = Entry(frame1, textvariable=OP_PID_temp, justify=CENTER, font=("calibri", 18), width=15)
OP_PID_temperatura.pack()
OP_PID_temperatura.place(x=1000, y=50)

OP_PID_phh = Entry(frame1, textvariable=OP_PID_ph, justify=CENTER, font=("calibri", 18), width=15)
OP_PID_phh.pack()
OP_PID_phh.place(x=1000, y=100)

OP_PID_odd = Entry(frame1, textvariable=OP_PID_od, justify=CENTER, font=("calibri", 18), width=15)
OP_PID_odd.pack()
OP_PID_odd.place(x=1000, y=150)

OP_PID_frecuencia = Entry(frame1, textvariable=OP_PID_frec, justify=CENTER, font=("calibri", 18), width=15)
OP_PID_frecuencia.pack()
OP_PID_frecuencia.place(x=1000, y=200)

########################### GRAFICA VALORES INSTANTANEOS

canvas = FigureCanvasTkAgg(figg, master=frame1)
canvas.get_tk_widget().pack(side="bottom", fill="x")
figg.canvas.draw()


###########################   BUTTON FRAME 1
def _quit():
    output_temp.start(0)  #
    output_frec.start(0)
    output_ph.SetPWM(0, 0, 0)  # duty =0,estado =0 (OFF) , signo =0 (+)
    GPIO.output(31, False)  # bomba1
    GPIO.output(33, False)  # bomba2
    GPIO.output(35, False)  # bomba3
    GPIO.output(37, False)  # bomba4
    GPIO.output(16, False)  # reset rene

    pp.quit()
    pp.destroy()


Salir = Button(frame1, text='SALIR', command=_quit, font=("calibri", 18), width=15)
Salir.pack()
Salir.place(x=1000, y=300)


def ENSAYO_START():
    global ban_grafica
    global label_state_registro
    global label_color_state_reg
    global ban_registrando

    ban_grafica = 1
    if ban_registrando == 0:
        label_state_registro = "REGISTRANDO"
        label_color_t = "green"
        MODO_reg.set(label_state_registro)
        MODO_registro.config(bg=label_color_t)
        ban_registrando = 1
    else:
        # ban_grafica=0
        label_state_registro = "SIN REGISTRAR"
        label_color_t = "red"
        MODO_reg.set(label_state_registro)
        MODO_registro.config(bg=label_color_t)
        ban_registrando = 0


graph_inicio = Button(frame1, text='REGISTRO', command=ENSAYO_START, font=("calibri", 18), width=13)
graph_inicio.pack()
graph_inicio.place(x=5, y=250)


def temperatura_setting():
    global SP_PID_temp
    global OP_PID_temp
    global state_t

    def C_off_Temp():
        global label_state_t
        global label_color_t
        global state_t

        if label_state_t == "ON":
            label_state_t = 'OFF'
            label_color_t = "red"
            C_off_Temp.config(text=label_state_t)
            C_off_Temp.config(bg=label_color_t)
            MODO_temp.set(label_state_t)
            MODO_temperatura.config(bg=label_color_t)
        else:
            label_state_t = 'ON'
            label_color_t = "green"
            C_off_Temp.config(text=label_state_t)
            C_off_Temp.config(bg=label_color_t)
            MODO_temp.set(label_state_t + " - " + label_state_auto_t)
            MODO_temperatura.config(bg=label_color_t)

    def C_Auto_Temp():
        global label_state_auto_t
        global state_t
        if label_state_auto_t == "AUTO":
            label_state_auto_t = 'MANUAL'
            state_t = 'normal'
            C_Auto_Temp.config(text=label_state_auto_t)
            scale_OP_temp.config(state=state_t)
            OPT1.config(state=state_t)
        else:
            label_state_auto_t = 'AUTO'
            state_t = 'disabled'
            C_Auto_Temp.config(text=label_state_auto_t)
            scale_OP_temp.config(state=state_t)
            OPT1.config(state=state_t)
        if label_state_t == "ON":
            MODO_temp.set(label_state_t + " - " + label_state_auto_t)

    def _quit_temp():
        ps_setting_temperatura.destroy()

    ps_setting_temperatura = Tkinter.Toplevel(pp)
    ps_setting_temperatura.geometry('1000x600')
    ps_setting_temperatura.wm_title("TEMPERATURA")
    sub_FramePID = Frame(ps_setting_temperatura, highlightbackground="blue")
    sub_FramePID.pack()
    sub_FramePID.place(height=300, width=1000, x=0, y=0)
    sub_FramePID.config(background="white")
    sub_Frame_k_PID = Frame(ps_setting_temperatura, relief=RAISED)
    sub_Frame_k_PID.pack()
    sub_Frame_k_PID.place(height=300, width=1000, x=0, y=300)

    borde_FramePID = Frame(sub_FramePID, relief=GROOVE, borderwidth=4, width=990, height=290)
    borde_FramePID_label = Label(borde_FramePID, text="CONFIGURACION - TEMPERATURA", font=("calibri", 18),
                                 width=28).place(relx=.066, rely=0.04, anchor=W)
    borde_FramePID.pack(anchor=NW)
    borde_FramePID.place(x=5, y=5)

    borde_Frame_k_PID = Frame(sub_Frame_k_PID, relief=GROOVE, borderwidth=4, width=990, height=290)
    borde_Frame_k_PID_label = Label(borde_Frame_k_PID, text="PARAMETROS PID", font=("calibri", 18), width=16).place(
        relx=.066, rely=0.04, anchor=W)
    borde_Frame_k_PID.pack(anchor=NW)
    borde_Frame_k_PID.place(x=5, y=5)

    variable = Label(sub_FramePID, text="SP °C", font=("calibri", 18), width=14)

    variable.pack()
    variable.place(x=30, y=50)

    variable = Label(sub_FramePID, text="SALIDA PID %", font=("calibri", 18), width=14)

    variable.pack()
    variable.place(x=200, y=50)

    scale_temperatura = Scale(sub_FramePID, variable=SP_PID_temp, activebackground="blue", resolution=0.1, takefocus=0,
                              showvalue=0)

    scale_temperatura.pack()
    scale_temperatura.place(x=120, y=90)

    scale_OP_temp = Scale(sub_FramePID, variable=OP_PID_temp, resolution=0.1, activebackground="blue", takefocus=0,
                          state=state_t, showvalue=0)
    scale_OP_temp.pack()
    scale_OP_temp.place(x=285, y=90)

    SPT1 = Entry(sub_FramePID, textvariable=SP_PID_temp, justify=CENTER, font=("calibri", 18), width=12)
    SPT1.pack()
    SPT1.place(x=30, y=220)

    OPT1 = Entry(sub_FramePID, textvariable=OP_PID_temp, state=state_t, justify=CENTER, font=("calibri", 18), width=12)
    OPT1.pack()
    OPT1.place(x=210, y=220)

    C_off_Temp = Button(sub_FramePID, text=label_state_t, bg=label_color_t, command=C_off_Temp, font=("calibri", 18),
                        width=10)
    C_off_Temp.pack()
    C_off_Temp.place(x=470, y=110)

    C_Auto_Temp = Button(sub_FramePID, text=label_state_auto_t, command=C_Auto_Temp, font=("calibri", 18), width=10)
    C_Auto_Temp.pack()
    C_Auto_Temp.place(x=670, y=110)

    Salir_temp = Button(borde_Frame_k_PID, text='SALIR', command=_quit_temp, font=("calibri", 18), width=12)
    Salir_temp.pack()
    Salir_temp.place(x=700, y=200)

    variable = Label(borde_Frame_k_PID, text="Kp", font=("calibri", 18), width=10)
    variable.pack()
    variable.place(x=25, y=50)

    valori = Label(borde_Frame_k_PID, text="Ki", font=("calibri", 18), width=10)
    valori.pack()
    valori.place(x=175, y=50)

    ac = Label(borde_Frame_k_PID, text="Kd", font=("calibri", 18), width=10)
    ac.pack()
    ac.place(x=325, y=50)

    P1 = Entry(borde_Frame_k_PID, textvariable=P_t, justify=CENTER, font=("calibri", 18), width=10)
    P1.pack()
    P1.place(x=25, y=100)

    I1 = Entry(borde_Frame_k_PID, textvariable=I_t, justify=CENTER, font=("calibri", 18), width=10)
    I1.pack()
    I1.place(x=175, y=100)

    D1 = Entry(borde_Frame_k_PID, textvariable=D_t, justify=CENTER, font=("calibri", 18), width=10)
    D1.pack()
    D1.place(x=325, y=100)


def ph_setting():
    global state_ph

    def C_off_ph():
        global label_state_ph
        global label_color_ph
        global state_PWM_od

        if label_state_ph == "ON":
            label_state_ph = 'OFF'
            label_color_ph = "red"
            C_off_ph.config(text=label_state_ph)
            C_off_ph.config(bg=label_color_ph)
            MODO_ph.set(label_state_ph)
            MODO_phh.config(bg=label_color_ph)

        else:
            label_state_ph = 'ON'
            label_color_ph = "green"
            C_off_ph.config(text=label_state_ph)
            C_off_ph.config(bg=label_color_ph)
            MODO_ph.set(label_state_ph + " - " + label_state_auto_ph)
            MODO_phh.config(bg=label_color_ph)

    def C_Auto_ph():
        global label_state_auto_ph
        global state_ph

        if label_state_auto_ph == "AUTO":
            label_state_auto_ph = 'MANUAL'
            state_ph = 'normal'
            C_Auto_ph.config(text=label_state_auto_ph)
            scale_OP_ph.config(state=state_ph)
            OPPH1.config(state=state_ph)
        else:
            label_state_auto_ph = 'AUTO'
            state_ph = 'disabled'
            C_Auto_ph.config(text=label_state_auto_ph)
            scale_OP_ph.config(state=state_ph)
            OPPH1.config(state=state_ph)
        if label_state_ph == "ON":
            MODO_ph.set(label_state_ph + " - " + label_state_auto_ph)

    def _quit_ph():
        ps_setting_ph.destroy()

    ps_setting_ph = Tkinter.Toplevel(pp)
    ps_setting_ph.geometry('1000x600')
    ps_setting_ph.wm_title("SETTING PH")
    global SP_PID_ph
    global OP_PID_ph

    sub_FramePID = Frame(ps_setting_ph, highlightbackground="blue")
    sub_FramePID.pack()
    sub_FramePID.place(height=300, width=1000, x=0, y=0)
    sub_FramePID.config(background="white")
    sub_Frame_k_PID = Frame(ps_setting_ph, relief=RAISED)
    sub_Frame_k_PID.pack()
    sub_Frame_k_PID.place(height=300, width=1000, x=0, y=300)

    borde_FramePID = Frame(sub_FramePID, relief=GROOVE, borderwidth=4, width=990, height=290)
    borde_FramePID_label = Label(borde_FramePID, text="CONFIGURACION - PH", font=("calibri", 18), width=28).place(
        relx=.066, rely=0.04, anchor=W)
    borde_FramePID.pack(anchor=NW)
    borde_FramePID.place(x=5, y=5)

    borde_Frame_k_PID = Frame(sub_Frame_k_PID, relief=GROOVE, borderwidth=4, width=990, height=290)
    borde_Frame_k_PID_label = Label(borde_Frame_k_PID, text="PARAMETROS PID", font=("calibri", 18), width=16).place(
        relx=.066, rely=0.04, anchor=W)
    borde_Frame_k_PID.pack(anchor=NW)
    borde_Frame_k_PID.place(x=5, y=5)

    variable = Label(borde_FramePID, text="SP PH", font=("calibri", 18), width=14)
    variable.pack()
    variable.place(x=30, y=50)

    variable = Label(borde_FramePID, text="SALIDA PID %", font=("calibri", 18), width=14)
    variable.pack()
    variable.place(x=200, y=50)

    scale = Scale(borde_FramePID, variable=SP_PID_ph, resolution=-0.1, from_=0, to=14, activebackground="blue",
                  takefocus=0, showvalue=0)
    scale.pack()
    scale.place(x=120, y=90)

    scale_OP_ph = Scale(borde_FramePID, variable=OP_PID_ph, activebackground="blue", from_=-100, to=100, takefocus=0,
                        state=state_ph, showvalue=0, resolution=0.1)
    scale_OP_ph.pack()
    scale_OP_ph.place(x=285, y=90)

    SPPH1 = Entry(borde_FramePID, textvariable=SP_PID_ph, justify=CENTER, font=("calibri", 18), width=12)
    SPPH1.pack()
    SPPH1.place(x=30, y=220)

    OPPH1 = Entry(borde_FramePID, textvariable=OP_PID_ph, state=state_ph, justify=CENTER, font=("calibri", 18),
                  width=12)
    OPPH1.pack()
    OPPH1.place(x=210, y=220)

    C_off_ph = Button(borde_FramePID, text=label_state_ph, command=C_off_ph, bg=label_color_ph, font=("calibri", 18),
                      width=10)
    C_off_ph.pack()
    C_off_ph.place(x=470, y=110)

    C_Auto_ph = Button(borde_FramePID, text=label_state_auto_ph, command=C_Auto_ph, font=("calibri", 18), width=10)
    C_Auto_ph.pack()
    C_Auto_ph.place(x=670, y=110)

    Salir_ph = Button(borde_Frame_k_PID, text='SALIR', command=_quit_ph, font=("calibri", 18), width=12)
    Salir_ph.pack()
    Salir_ph.place(x=700, y=200)

    variable = Label(borde_Frame_k_PID, text="Kp", font=("calibri", 18), width=10)
    variable.pack()
    variable.place(x=25, y=50)

    valori = Label(borde_Frame_k_PID, text="Ki", font=("calibri", 18), width=10)
    valori.pack()
    valori.place(x=175, y=50)

    ac = Label(borde_Frame_k_PID, text="Kd", font=("calibri", 18), width=10)
    ac.pack()
    ac.place(x=325, y=50)

    P1 = Entry(borde_Frame_k_PID, textvariable=P_ph, justify=CENTER, font=("calibri", 18), width=10)
    P1.pack()
    P1.place(x=25, y=100)

    I1 = Entry(borde_Frame_k_PID, textvariable=I_ph, justify=CENTER, font=("calibri", 18), width=10)
    I1.pack()
    I1.place(x=175, y=100)

    D1 = Entry(borde_Frame_k_PID, textvariable=D_ph, justify=CENTER, font=("calibri", 18), width=10)
    D1.pack()
    D1.place(x=325, y=100)


def od_setting():
    global state_od
    global state_f

    def C_off_od():
        global label_state_od
        global label_color_od
        global label_state_f
        global label_color_f

        if label_state_od == "CASCADA":
            label_state_od = 'OFF'
            label_color_od = "red"
            C_off_od.config(text=label_state_od)
            C_off_od.config(bg=label_color_od)
            MODO_od.set(label_state_od)
            MODO_odd.config(bg=label_color_od)
            if label_state_od == "OFF":
                label_state_f = 'ON'
                label_color_f = "green"
                MODO_frec.set(label_state_f)
                MODO_frecuencia.config(bg=label_color_f)
        else:
            GPIO.output(PIN_ENCENDIDO_MOTOR, True)
            label_state_od = 'CASCADA'
            label_color_od = "green"
            C_off_od.config(text=label_state_od)
            C_off_od.config(bg=label_color_od)
            MODO_od.set(label_state_od + " - " + label_state_auto_od)
            MODO_odd.config(bg=label_color_od)
            if label_state_f == "ON":
                label_state_f = 'ON - CASCADA'
                label_color_f = "green"
                MODO_frec.set(label_state_f)
                MODO_frecuencia.config(bg=label_color_f)

    def _quit_od():
        ps_setting_od.destroy()

    ps_setting_od = Tkinter.Toplevel(pp)
    ps_setting_od.geometry('1000x600')
    ps_setting_od.wm_title("CONFIGURACION OX.DISUELTO")
    global SP_PID_od
    global OP_PID_od

    sub_FramePID = Frame(ps_setting_od, highlightbackground="blue")
    sub_FramePID.pack()
    sub_FramePID.place(height=300, width=1000, x=0, y=0)
    sub_FramePID.config(background="white")
    sub_Frame_k_PID = Frame(ps_setting_od, relief=RAISED)
    sub_Frame_k_PID.pack()
    sub_Frame_k_PID.place(height=300, width=1000, x=0, y=300)

    borde_FramePID = Frame(sub_FramePID, relief=GROOVE, borderwidth=4, width=990, height=290)
    borde_FramePID_label = Label(borde_FramePID, text="CONFIGURACION - OD", font=("calibri", 18), width=28).place(
        relx=.066, rely=0.04, anchor=W)
    borde_FramePID.pack(anchor=NW)
    borde_FramePID.place(x=5, y=5)

    borde_Frame_k_PID = Frame(sub_Frame_k_PID, relief=GROOVE, borderwidth=4, width=990, height=290)
    borde_Frame_k_PID_label = Label(borde_Frame_k_PID, text="PARAMETROS PID", font=("calibri", 18), width=16).place(
        relx=.066, rely=0.04, anchor=W)
    borde_Frame_k_PID.pack(anchor=NW)
    borde_Frame_k_PID.place(x=5, y=5)

    variable = Label(borde_FramePID, text="SP OD", font=("calibri", 18), width=14)
    variable.pack()
    variable.place(x=30, y=50)

    variable = Label(borde_FramePID, text="SALIDA PID %", font=("calibri", 18), width=14)
    variable.pack()
    variable.place(x=200, y=50)

    scale = Scale(borde_FramePID, variable=SP_PID_od, activebackground="blue", takefocus=0, showvalue=0)
    scale.pack()
    scale.place(x=120, y=90)

    scale_OP_od = Scale(borde_FramePID, variable=OP_PID_od, resolution=0.1, state=state_od, activebackground="blue",
                        takefocus=0, showvalue=0)
    scale_OP_od.pack()
    scale_OP_od.place(x=285, y=90)

    SPOD1 = Entry(borde_FramePID, textvariable=SP_PID_od, justify=CENTER, font=("calibri", 18), width=12)
    SPOD1.pack()
    SPOD1.place(x=30, y=220)

    OPOD1 = Entry(borde_FramePID, textvariable=OP_PID_od, state=state_od, justify=CENTER, font=("calibri", 18),
                  width=12)
    OPOD1.pack()
    OPOD1.place(x=210, y=220)

    C_off_od = Button(borde_FramePID, text=label_state_od, command=C_off_od, bg=label_color_od, font=("calibri", 18),
                      width=10)
    C_off_od.pack()
    C_off_od.place(x=470, y=110)

    Salir_od = Button(borde_Frame_k_PID, text='SALIR', command=_quit_od, font=("calibri", 18), width=12)
    Salir_od.pack()
    Salir_od.place(x=700, y=200)

    variable = Label(borde_Frame_k_PID, text="Kp", font=("calibri", 18), width=10)
    variable.pack()
    variable.place(x=25, y=50)

    # variable=Label(borde_Frame_k_PID,text="Kod",font=("calibri",18),width=10)
    # variable.pack()
    # variable.place(x=25, y=150)

    valori = Label(borde_Frame_k_PID, text="Ki", font=("calibri", 18), width=10)
    valori.pack()
    valori.place(x=175, y=50)

    ac = Label(borde_Frame_k_PID, text="Kd", font=("calibri", 18), width=10)
    ac.pack()
    ac.place(x=325, y=50)

    P1 = Entry(borde_Frame_k_PID, textvariable=P_od, justify=CENTER, font=("calibri", 18), width=10)
    P1.pack()
    P1.place(x=25, y=100)

    # Pod=Entry(borde_Frame_k_PID,textvariable=P_od,justify=CENTER,font=("calibri",18),width=10)
    # Pod.pack()
    # Pod.place(x=25, y=200)

    I1 = Entry(borde_Frame_k_PID, textvariable=I_od, justify=CENTER, font=("calibri", 18), width=10)
    I1.pack()
    I1.place(x=175, y=100)

    D1 = Entry(borde_Frame_k_PID, textvariable=D_od, justify=CENTER, font=("calibri", 18), width=10)
    D1.pack()
    D1.place(x=325, y=100)


def frec_setting():
    global SP_PID_frec
    global OP_PID_frec
    global state_f

    def C_off_frec():
        global label_state_f
        global label_color_f
        
        
        if label_state_f == "ON":
            GPIO.output(PIN_ENCENDIDO_MOTOR, False)
            label_state_f = 'OFF'
            label_color_f = "red"
            C_off_frec.config(text=label_state_f)
            C_off_frec.config(bg=label_color_f)
            MODO_frec.set(label_state_f)
            MODO_frecuencia.config(bg=label_color_f)
        else:
            #Cambio
            GPIO.output(PIN_ENCENDIDO_MOTOR, True)
            label_state_f = 'ON'
            label_color_f = "green"
            C_off_frec.config(text=label_state_f)
            C_off_frec.config(bg=label_color_f)
            MODO_frec.set(label_state_f + " - " + label_state_auto_f)
            MODO_frecuencia.config(bg=label_color_f)

    def C_Auto_frec():
        global label_state_auto_f
        global state_f
        if label_state_auto_f == "AUTO":
            label_state_auto_f = 'MANUAL'
            state_f = 'normal'
            scale_OP_frec.config(state=state_f)
            OPFREC1.config(state=state_f)
            C_Auto_frec.config(text=label_state_auto_f)
            if label_state_f == "ON":
                MODO_frec.set("ON - " + label_state_auto_f)
        else:
            label_state_auto_f = 'AUTO'
            state_f = 'disabled'
            C_Auto_frec.config(text=label_state_auto_f)
            scale_OP_frec.config(state=state_f)
            OPFREC1.config(state=state_f)
            if label_state_f == "ON":
                MODO_frec.set("ON - " + label_state_auto_f)

    def _quit_frec():
        ps_setting_frec.destroy()

    ps_setting_frec = Tkinter.Toplevel(pp)
    ps_setting_frec.geometry('1000x600')
    ps_setting_frec.wm_title("CONFIGURACION RPM")

    sub_FramePID = Frame(ps_setting_frec, highlightbackground="blue")
    sub_FramePID.pack()
    sub_FramePID.place(height=300, width=1000, x=0, y=0)
    sub_FramePID.config(background="white")
    sub_Frame_k_PID = Frame(ps_setting_frec, relief=RAISED)
    sub_Frame_k_PID.pack()
    sub_Frame_k_PID.place(height=300, width=1000, x=0, y=300)

    borde_FramePID = Frame(sub_FramePID, relief=GROOVE, borderwidth=4, width=990, height=290)
    borde_FramePID_label = Label(borde_FramePID, text="CONFIGURACION - RPM", font=("calibri", 18), width=28).place(
        relx=.066, rely=0.04, anchor=W)
    borde_FramePID.pack(anchor=NW)
    borde_FramePID.place(x=5, y=5)

    borde_Frame_k_PID = Frame(sub_Frame_k_PID, relief=GROOVE, borderwidth=4, width=990, height=290)
    borde_Frame_k_PID_label = Label(borde_Frame_k_PID, text="PARAMETROS PID", font=("calibri", 18), width=16).place(
        relx=.066, rely=0.04, anchor=W)
    borde_Frame_k_PID.pack(anchor=NW)
    borde_Frame_k_PID.place(x=5, y=5)

    variable = Label(borde_FramePID, text="SP RPM", font=("calibri", 18), width=14)
    variable.pack()
    variable.place(x=30, y=50)

    variable = Label(borde_FramePID, text="SALIDA PID %", font=("calibri", 18), width=14)
    variable.pack()
    variable.place(x=200, y=50)

    global SP_PID_frec
    global OP_PID_frec
    scale_frec = Scale(borde_FramePID, variable=SP_PID_frec, activebackground="blue", from_=0, to=1400, takefocus=0,
                       showvalue=0)
    scale_frec.pack()
    scale_frec.place(x=120, y=90)

    scale_OP_frec = Scale(borde_FramePID, variable=OP_PID_frec, activebackground="blue", from_=0, to=100, takefocus=0,
                          state=state_f, showvalue=0)
    scale_OP_frec.pack()
    scale_OP_frec.place(x=285, y=90)

    SPFREC1 = Entry(borde_FramePID, textvariable=SP_PID_frec, justify=CENTER, font=("calibri", 18), width=12)
    SPFREC1.pack()
    SPFREC1.place(x=30, y=220)

    OPFREC1 = Entry(borde_FramePID, textvariable=OP_PID_frec, state=state_f, justify=CENTER, font=("calibri", 18),
                    width=12)
    OPFREC1.pack()
    OPFREC1.place(x=210, y=220)

    C_off_frec = Button(borde_FramePID, text=label_state_f, bg=label_color_f, command=C_off_frec, font=("calibri", 18),
                        width=10)
    C_off_frec.pack()
    C_off_frec.place(x=470, y=110)

    C_Auto_frec = Button(borde_FramePID, text=label_state_auto_f, command=C_Auto_frec, font=("calibri", 18), width=10)
    C_Auto_frec.pack()
    C_Auto_frec.place(x=670, y=110)

    Salir_frec = Button(borde_Frame_k_PID, text='SALIR', command=_quit_frec, font=("calibri", 18), width=12)
    Salir_frec.pack()
    Salir_frec.place(x=700, y=200)

    variable = Label(borde_Frame_k_PID, text="Kp", font=("calibri", 18), width=10)
    variable.pack()
    variable.place(x=25, y=50)

    valori = Label(borde_Frame_k_PID, text="Ki", font=("calibri", 18), width=10)
    valori.pack()
    valori.place(x=175, y=50)

    ac = Label(borde_Frame_k_PID, text="Kd", font=("calibri", 18), width=10)
    ac.pack()
    ac.place(x=325, y=50)

    P1 = Entry(borde_Frame_k_PID, textvariable=P_frec, justify=CENTER, font=("calibri", 18), width=10)
    P1.pack()
    P1.place(x=25, y=100)

    I1 = Entry(borde_Frame_k_PID, textvariable=I_frec, justify=CENTER, font=("calibri", 18), width=10)
    I1.pack()
    I1.place(x=175, y=100)

    D1 = Entry(borde_Frame_k_PID, textvariable=D_frec, justify=CENTER, font=("calibri", 18), width=10)
    D1.pack()
    D1.place(x=325, y=100)


def TEST():
    global VI_frec
    global VI_t
    global state_f
    global ban_TEST
    global output_temp
    global output_frec
    ban_TEST = 1

    def refreshSALIR(ban_TEST):
        while (1):
            time.sleep(0.5)
            Salir_TEST.config(bg="RED")
            time.sleep(0.5)
            Salir_TEST.config(bg="WHITE")

    def _quit_TEST():
        global ban_TEST
        ban_TEST = 0
        ps_Test.destroy()

    def reset_rene():
        GPIO.output(16, True)  ## Enciendo
        time.sleep(2)  ## Esperamos
        GPIO.output(16, False)

    def bomba1():
        global ban_output_bomba1
        if ban_output_bomba1 == 0:
            GPIO.output(31, True)  ## Enciendo
            ban_output_bomba1 = 1
            MODO_label_B1 = "ON"
            label_color_B1 = "green"
            MODO_B1_label.set(MODO_label_B1)
            MODO_B1.config(bg=label_color_B1)
        else:
            GPIO.output(31, False)
            ban_output_bomba1 = 0
            MODO_label_B1 = "OFF"
            label_color_B1 = "tomato"
            MODO_B1_label.set(MODO_label_B1)
            MODO_B1.config(bg=label_color_B1)

    def bomba2():
        global ban_output_bomba2
        if ban_output_bomba2 == 0:
            GPIO.output(33, True)  ## Enciendo
            ban_output_bomba2 = 1
            MODO_label_B2 = "ON"
            label_color_B2 = "green"
            MODO_B2_label.set(MODO_label_B2)
            MODO_B2.config(bg=label_color_B2)
        else:
            GPIO.output(33, False)
            ban_output_bomba2 = 0
            MODO_label_B2 = "OFF"
            label_color_B2 = "tomato"
            MODO_B2_label.set(MODO_label_B2)
            MODO_B2.config(bg=label_color_B2)

    def bomba3():
        global ban_output_bomba3
        if ban_output_bomba3 == 0:
            GPIO.output(35, True)  ## Enciendo
            ban_output_bomba3 = 1
            MODO_label_B3 = "ON"
            label_color_B3 = "green"
            MODO_B3_label.set(MODO_label_B3)
            MODO_B3.config(bg=label_color_B3)
        else:
            GPIO.output(35, False)
            ban_output_bomba3 = 0
            MODO_label_B3 = "OFF"
            label_color_B3 = "tomato"
            MODO_B3_label.set(MODO_label_B3)
            MODO_B3.config(bg=label_color_B3)

    def bomba4():
        global ban_output_bomba4
        if ban_output_bomba4 == 0:
            GPIO.output(37, True)  ## Enciendo
            ban_output_bomba4 = 1
            MODO_label_B4 = "ON"
            label_color_B4 = "green"
            MODO_B4_label.set(MODO_label_B4)
            MODO_B4.config(bg=label_color_B4)
        else:
            GPIO.output(37, False)
            ban_output_bomba4 = 0
            MODO_label_B4 = "OFF"
            label_color_B4 = "tomato"
            MODO_B4_label.set(MODO_label_B4)
            MODO_B4.config(bg=label_color_B4)

    def MOTOR():
        global ban_motor
        global output_frec
        if ban_motor == 0:
            output_frec.ChangeDutyCycle(100)
            ban_motor = 1
            MODO_label_MOTOR = "ON"
            label_color_MOTOR = "GREEN"
            MODO_MOTOR_label.set(MODO_label_MOTOR)
            MODO_MOTOR.config(bg=label_color_MOTOR)
        else:
            ban_motor = 0
            MODO_label_MOTOR = "OFF"
            label_color_MOTOR = "tomato"
            output_frec.ChangeDutyCycle(0)
            MODO_MOTOR_label.set(MODO_label_MOTOR)
            MODO_MOTOR.config(bg=label_color_MOTOR)

    def SOLENOIDE():
        global ban_solenoide
        if ban_solenoide == 0:
            GPIO.output(29, True)  ## Enciendo
            ban_solenoide = 1
            MODO_label_SOLENOIDE = "ON"
            label_color_SOLENOIDE = "GREEN"
            MODO_SOLENOIDE_label.set(MODO_label_SOLENOIDE)
            MODO_SOLENOIDE.config(bg=label_color_SOLENOIDE)
        else:
            GPIO.output(29, False)
            ban_solenoide = 0
            MODO_label_SOLENOIDE = "OFF"
            label_color_SOLENOIDE = "tomato"
            MODO_SOLENOIDE_label.set(MODO_label_SOLENOIDE)
            MODO_SOLENOIDE.config(bg=label_color_SOLENOIDE)

    def CALEFACTOR():
        global ban_calefactor
        global output_temp
        if ban_calefactor == 0:
            ban_calefactor = 1
            output_temp.ChangeDutyCycle(100)
            MODO_label_CALEFACTOR = "ON"
            label_color_CALEFACTOR = "GREEN"
            MODO_CALEFACTOR_label.set(MODO_label_CALEFACTOR)
            MODO_CALEFACTOR.config(bg=label_color_CALEFACTOR)

        else:
            ban_calefactor = 0
            MODO_label_CALEFACTOR = "OFF"
            label_color_CALEFACTOR = "tomato"
            output_temp.ChangeDutyCycle(0)
            MODO_CALEFACTOR_label.set(MODO_label_CALEFACTOR)
            MODO_CALEFACTOR.config(bg=label_color_CALEFACTOR)

    ps_Test = Tkinter.Toplevel(pp)
    ps_Test.geometry('1000x600')
    ps_Test.wm_title("Panel de Prueba")

    borde_FramePID = Frame(ps_Test, relief=GROOVE, borderwidth=4, width=990, height=590)
    borde_FramePID_label = Label(ps_Test, text="PURGA", font=("calibri", 18), width=28).place(relx=.066, rely=0.04,
                                                                                             anchor=W)
    borde_FramePID.pack(anchor=NW)
    borde_FramePID.place(x=5, y=5)

    Salir_TEST = Button(ps_Test, text='SALIR ', bg="RED", command=_quit_TEST, font=("calibri", 25), width=16)
    Salir_TEST.pack()
    Salir_TEST.place(x=600, y=470)

    bomba_1 = Button(ps_Test, text='NUTRIENTE', command=bomba1, font=("calibri", 18), width=15)
    bomba_1.pack()
    bomba_1.place(x=15, y=50)

    bomba_2 = Button(ps_Test, text='BASE', command=bomba2, font=("calibri", 18), width=15)
    bomba_2.pack()
    bomba_2.place(x=15, y=100)

    bomba_3 = Button(ps_Test, text='ÁCIDO', command=bomba3, font=("calibri", 18), width=15)
    bomba_3.pack()
    bomba_3.place(x=15, y=150)

    #bomba_4 = Button(ps_Test, text='BOMBA 4', command=bomba4,font=("calibri",18),width=15)
    #bomba_4.pack()
    #bomba_4.place(x=15, y=360)

    #MOTOR = Button(ps_Test, text='MOTOR', command=MOTOR, font=("calibri", 18), width=15)
    #MOTOR.pack()
    #MOTOR.place(x=15, y=200)

    SOLENOIDE = Button(ps_Test, text='SOLENOIDE', command=SOLENOIDE, font=("calibri", 18), width=15)
    SOLENOIDE.pack()
    SOLENOIDE.place(x=15, y=250)

    CALEFACTOR = Button(ps_Test, text='CALEFACTOR', command=CALEFACTOR, font=("calibri", 18), width=15)
    CALEFACTOR.pack()
    CALEFACTOR.place(x=15, y=200)

    MODO_B1 = Entry(ps_Test, textvariable=MODO_B1_label, bg=label_color_B1, font=("calibri", 18), width=15,
                    justify=CENTER)
    MODO_B1.pack()
    MODO_B1.place(x=300, y=50)
    MODO_B1_label.set(MODO_label_B1)

    MODO_B2 = Entry(ps_Test, textvariable=MODO_B2_label, font=("calibri", 18), bg=label_color_B2, width=15,
                    justify=CENTER)
    MODO_B2.pack()
    MODO_B2.place(x=300, y=100)
    MODO_B2_label.set(MODO_label_B2)

    MODO_B3 = Entry(ps_Test, textvariable=MODO_B3_label, justify=CENTER, bg=label_color_B3, font=("calibri", 18),
                    width=15)
    MODO_B3.pack()
    MODO_B3.place(x=300, y=150)
    MODO_B3_label.set(MODO_label_B3)

    # MODO_B4=Entry(ps_Test,textvariable=MODO_B4_label,bg=label_color_B4,justify=CENTER,font=("calibri",18),width=15)
    # MODO_B4.pack()
    # MODO_B4.place(x=300, y=200)
    # MODO_B4_label.set(MODO_label_B4)

    MODO_MOTOR = Entry(ps_Test, textvariable=MODO_MOTOR_label, bg=label_color_MOTOR, justify=CENTER,
                       font=("calibri", 18), width=15)
    MODO_MOTOR.pack()
    MODO_MOTOR.place(x=300, y=200)
    MODO_MOTOR_label.set(MODO_label_MOTOR)

    MODO_SOLENOIDE = Entry(ps_Test, textvariable=MODO_SOLENOIDE_label, bg=label_color_SOLENOIDE, justify=CENTER,
                           font=("calibri", 18), width=15)
    MODO_SOLENOIDE.pack()
    MODO_SOLENOIDE.place(x=300, y=250)
    MODO_SOLENOIDE_label.set(MODO_label_SOLENOIDE)

    MODO_CALEFACTOR = Entry(ps_Test, textvariable=MODO_CALEFACTOR_label, bg=label_color_CALEFACTOR, justify=CENTER,
                            font=("calibri", 18), width=15)
    MODO_CALEFACTOR.pack()
    MODO_CALEFACTOR.place(x=300, y=200)
    MODO_CALEFACTOR_label.set(MODO_label_CALEFACTOR)

    #VI_frecuencia = Entry(ps_Test, textvariable=VI_frec, justify=CENTER, font=("calibri", 18), width=15)
    #VI_frecuencia.pack()
    #VI_frecuencia.place(x=575, y=200)

    VI_temperatura = Entry(ps_Test, textvariable=VI_t, justify=CENTER, font=("calibri", 18), width=15)
    VI_temperatura.pack()
    VI_temperatura.place(x=575, y=200)

    reset_r = Button(ps_Test, text='RESET COMUNICACION', command=reset_rene, font=("calibri", 18), width=24)
    reset_r.pack()
    reset_r.place(x=15, y=350)
    
    #t = thread.start_new_thread(refreshSALIR, (T,))


B2 = Button(frame1, text="Temperatura", command=temperatura_setting, font=("calibri", 18), width=13)
B2.pack()
B2.place(x=5, y=50)

B3 = Button(frame1, text="PH", command=ph_setting, font=("calibri", 18), width=13)
B3.pack()
B3.place(x=5, y=100)

B4 = Button(frame1, text="OD", command=od_setting, font=("calibri", 18), width=13)
B4.pack()
B4.place(x=5, y=150)

B1 = Button(frame1, text="RPM", command=frec_setting, font=("calibri", 18), width=13)
B1.pack()
B1.place(x=5, y=200)

B5 = Button(frame1, text="PURGA", command=TEST, font=("calibri", 18), width=13)
B5.pack()
B5.place(x=5, y=300)


##################################FRAME 2 - Calibracion ################################
########
VI_t_h = 1000
VI_t_l = 0

def tilde_t_alto():
    global VI_t_h  ### Agregado
    if var_cal_temp_h.get():
        REF_cal_temp_h.config(state=DISABLED)
        VI_t_h = float(VI_cal_temp_h.get())
        print("H: ", VI_t_h)
        if var_cal_temp_l.get():
            CONFIRMAR_cal_t.config(state=NORMAL)
    else:
        CONFIRMAR_cal_t.config(state=DISABLED)
        REF_cal_temp_h.config(state=NORMAL)

def tilde_t_bajo():
    global VI_t_l  ### Agregado
    if var_cal_temp_l.get():
        REF_cal_temp_l.config(state=DISABLED)
        VI_t_l = float(VI_cal_temp_l.get())
        print("L: ", VI_t_l)
        if var_cal_temp_h.get():
            CONFIRMAR_cal_t.config(state=NORMAL)
    else:
        CONFIRMAR_cal_t.config(state=DISABLED)
        REF_cal_temp_l.config(state=NORMAL)
    


def confirmar_cal_temp():
    global VI_t
    global pendiente
    global p_cal_th
    global p_cal_phh
    global p_cal_odh
    global p_cal_frech
    global p_cal_frecl
    global MODO_config_temp
    global MODO_config_ph
    global MODO_config_od
    global MODO_config_frec
    global REF_phh
    global REF_ohh
    global REF_frech
    global REF_th
    global REF_tl
    global VI_t_l
    global VI_t_h
    global CONFIRMAR_cal_t
    
    VI_cal_temp_l.config(state=NORMAL)
    REF_cal_temp_l.config(state=NORMAL)
    VI_cal_temp_h.config(state=NORMAL)
    REF_cal_temp_h.config(state=NORMAL)
    CONFIRMAR_cal_t.config(state=DISABLED)
    tilde_t_h.deselect()
    tilde_t_l.deselect()
    if MODO_config_temp.get() == 1:
        with lock:
            modulo_sensado.set_slope("T", REF_th.get(), REF_tl.get(), VI_t_h,VI_t_l, p_cal_th.get(), p_cal_tl.get())
        MODO_config_temp.set(0)

    if MODO_config_ph.get() == 1:
        with lock:
            modulo_sensado.set_slope("ph", REF_phh.get(),REF_phl.get(), VI_t_h,VI_t_l, p_cal_phh.get(), p_cal_phl.get())
        MODO_config_ph.set(0)

    if MODO_config_od.get() == 1:
        with lock:
            modulo_sensado.set_slope("od", REF_odh.get(), REF_odl.get(), VI_t_h,VI_t_l, p_cal_odh.get(), p_cal_odl.get())
        MODO_config_od.set(0)

    if MODO_config_frec.get() == 1:
        with lock:
            modulo_sensado.set_slope("rpm", REF_frech.get(), REF_frecl.get(),VI_t_h,VI_t_l, p_cal_frech.get(), p_cal_frecl.get())
        MODO_config_frec.set(0)


def temperatura_cal():
    global MODO_config
    global MODO_config_temp
    
    tilde_t_h.config(state=NORMAL)
    tilde_t_l.config(state=NORMAL)
    VI_cal_temp_l.config(state=NORMAL)
    VI_cal_temp_h.config(state=NORMAL)
    REF_cal_temp_h.config(state=NORMAL)
    REF_cal_temp_l.config(state=NORMAL)
    
    VI_cal_temp_l.config(textvariable=VI_t)
    VI_cal_temp_h.config(textvariable=VI_t)
    REF_cal_temp_h.config(textvariable=REF_th)
    REF_cal_temp_l.config(textvariable=REF_tl)
    label_cal = Label(FrameDD, text=" TEMPERATURA", font=("calibri", 18), width=14)
    label_cal.pack()
    label_cal.place(x=200, y=40)
    VI_cal_temp_l.config(textvariable=VI_t)
    MODO_config = 3
    MODO_config_temp.set(1)
    MODO_config_ph.set(0)
    MODO_config_od.set(0)
    MODO_config_frec.set(0)


def ph_cal():
    global MODO_config
    global MODO_config_ph
    tilde_t_h.config(state=NORMAL)
    tilde_t_l.config(state=NORMAL)
    VI_cal_temp_l.config(state=NORMAL)
    VI_cal_temp_h.config(state=NORMAL)
    REF_cal_temp_h.config(state=NORMAL)
    REF_cal_temp_l.config(state=NORMAL)
    
    label_cal = Label(FrameDD, text="     PH     ", font=("calibri", 18), width=14)
    label_cal.pack()
    label_cal.place(x=200, y=40)
    VI_cal_temp_l.config(textvariable=VI_ph)
    VI_cal_temp_h.config(textvariable=VI_ph)
    REF_cal_temp_h.config(textvariable=REF_phh)
    REF_cal_temp_l.config(textvariable=REF_phl)
    MODO_config = 4
    MODO_config_ph.set(1)
    MODO_config_temp.set(0)
    MODO_config_od.set(0)
    MODO_config_frec.set(0)


def od_cal():
    global MODO_config
    global MODO_config_od
    tilde_t_h.config(state=NORMAL)
    tilde_t_l.config(state=NORMAL)
    VI_cal_temp_l.config(state=NORMAL)
    VI_cal_temp_h.config(state=NORMAL)
    REF_cal_temp_h.config(state=NORMAL)
    REF_cal_temp_l.config(state=NORMAL)
    
    label_cal = Label(FrameDD, text="     OD     ", font=("calibri", 18), width=14)
    label_cal.pack()
    label_cal.place(x=200, y=40)
    VI_cal_temp_l.config(textvariable=VI_od)
    VI_cal_temp_h.config(textvariable=VI_od)
    REF_cal_temp_h.config(textvariable=REF_odh)
    REF_cal_temp_l.config(textvariable=REF_odl)
    MODO_config = 5
    MODO_config_temp.set(0)
    MODO_config_ph.set(0)
    MODO_config_od.set(1)
    MODO_config_frec.set(0)


def frec_cal():
    global MODO_config
    global MODO_config_frec
    tilde_t_h.config(state=NORMAL)
    tilde_t_l.config(state=NORMAL)
    VI_cal_temp_l.config(state=NORMAL)
    VI_cal_temp_h.config(state=NORMAL)
    REF_cal_temp_h.config(state=NORMAL)
    REF_cal_temp_l.config(state=NORMAL)
    
    label_cal = Label(FrameDD, text="     RPM    ", font=("calibri", 18), width=14)
    label_cal.pack()
    label_cal.place(x=200, y=40)
    VI_cal_temp_h.config(textvariable=VI_frec)
    VI_cal_temp_l.config(textvariable=VI_frec)
    REF_cal_temp_h.config(textvariable=REF_frech)
    REF_cal_temp_l.config(textvariable=REF_frecl)
    MODO_config = 6
    MODO_config_temp.set(0)
    MODO_config_ph.set(0)
    MODO_config_od.set(0)
    MODO_config_frec.set(1)


def resetvalores():
    global MODO_config
    MODO_config = 7
    with lock:
            modulo_sensado.reset_slopes()
    MODO_config = 0


FrameB = Frame(frame2, relief=SUNKEN)
FrameB.pack()
FrameB.place(height=800, width=650, x=0, y=0)
# FrameB.config(background="white")
FrameDD = Frame(frame2, relief=RAISED)
FrameDD.pack()
FrameDD.place(height=800, width=650, x=650, y=0)

Resetvalores_cal_t = Button(FrameDD, text='VALORES PREDERMINADOS', command=resetvalores, font=("calibri", 18), width=22)
Resetvalores_cal_t.pack()
Resetvalores_cal_t.place(x=250, y=600)

xf = Frame(FrameDD, relief=GROOVE, borderwidth=4, width=630, height=420)
label_cal = Label(xf, text="CALIBRACION", font=("calibri", 18), width=14).place(relx=.066, rely=0.0125, anchor=W)
xf.pack(anchor=NW)
xf.place(x=5, y=15)

CONFIRMAR_cal_t = Button(FrameDD, text='CONFIRMAR', state=DISABLED, font=("calibri", 18), width=14,
                         command=confirmar_cal_temp)
CONFIRMAR_cal_t.pack()
CONFIRMAR_cal_t.place(x=225, y=300)

VI_cal_temp_h = Entry(FrameDD, textvariable=VI_t, justify=CENTER, font=("calibri", 18), width=8)
VI_cal_temp_h.pack()
VI_cal_temp_h.place(x=200, y=140)

REF_cal_temp_h = Entry(FrameDD, textvariable=REF_th, justify=CENTER, font=("calibri", 18), width=8)
REF_cal_temp_h.pack()
REF_cal_temp_h.place(x=400, y=140)

VI_cal_temp_l = Entry(FrameDD, textvariable=VI_t, justify=CENTER, font=("calibri", 18), width=8)
VI_cal_temp_l.pack()
VI_cal_temp_l.place(x=200, y=190)
REF_cal_temp_l = Entry(FrameDD, textvariable=REF_tl, justify=CENTER, font=("calibri", 18), width=8)
REF_cal_temp_l.pack()
REF_cal_temp_l.place(x=400, y=190)

var_cal_temp_h = BooleanVar()
var_cal_temp_l = BooleanVar()

tilde_t_l = Checkbutton(FrameDD, variable=var_cal_temp_l, command=tilde_t_bajo)
tilde_t_l.pack()
tilde_t_l.place(x=550, y=195)


tilde_t_h = Checkbutton(FrameDD, variable=var_cal_temp_h, command=tilde_t_alto)
tilde_t_h.pack()
tilde_t_h.place(x=550, y=145)

tilde_t_h.config(state=DISABLED)
tilde_t_l.config(state=DISABLED)
VI_cal_temp_l.config(state=DISABLED)
VI_cal_temp_h.config(state=DISABLED)
REF_cal_temp_h.config(state=DISABLED)
REF_cal_temp_l.config(state=DISABLED)

label_cal1 = Label(FrameDD, text="MEDICIÓN", font=("calibri", 18), width=10)
label_cal1.pack()
label_cal1.place(x=180, y=90)

label_cal2 = Label(FrameDD, text="REFERENCIA", font=("calibri", 18), width=10)
label_cal2.pack()
label_cal2.place(x=350, y=90)

label_cal3 = Label(FrameDD, text="V Alto", font=("calibri", 18), width=6)
label_cal3.pack()
label_cal3.place(x=40, y=140)

label_cal4 = Label(FrameDD, text="V Bajo", font=("calibri", 18), width=6)
label_cal4.pack()
label_cal4.place(x=40, y=190)

xf = Frame(FrameB, relief=GROOVE, borderwidth=4, width=630, height=420)
xf_label = Label(xf, text="VARIABLES", font=("calibri", 18), width=14).place(relx=.066, rely=0.0125, anchor=W)
xf.pack(anchor=NW)
xf.place(x=15, y=15)

B2_cal = Button(FrameB, text="Temperatura", command=temperatura_cal, font=("calibri", 18), width=14)
B2_cal.pack()
B2_cal.place(x=220, y=100)

B3_cal = Button(FrameB, text="PH", command=ph_cal, font=("calibri", 18), width=14)
B3_cal.pack()
B3_cal.place(x=220, y=150)

B4_cal = Button(FrameB, text="OD", command=od_cal, font=("calibri", 18), width=14)
B4_cal.pack()
B4_cal.place(x=220, y=200)

B1_cal = Button(FrameB, text="RPM", command=frec_cal, font=("calibri", 18), width=14)
B1_cal.pack()
B1_cal.place(x=220, y=250)

##################################FRAME 4 - BOMBAS ################################
######## calibracion BOMBAS

variable_label_CONFIG = Label(frame4, text="CALIBRACION BOMBAS", font=("calibri", 18), width=20)
variable_label_CONFIG.pack()
variable_label_CONFIG.place(x=0, y=0)

#####botones de configuracion de tiempo de muestreo

FrameB1 = Frame(frame4, highlightbackground="blue")
FrameB1.pack()
FrameB1.place(height=750, width=435, x=0, y=0)
FrameB1.config(background="white")

FrameB2 = Frame(frame4, highlightbackground="blue")
FrameB2.pack()
FrameB2.place(height=750, width=430, x=435, y=0)
FrameB2.config(background="white")

FrameB3 = Frame(frame4, highlightbackground="blue")
FrameB3.pack()
FrameB3.place(height=750, width=435, x=865, y=0)
FrameB3.config(background="white")

borde_B1 = Frame(FrameB1, relief=GROOVE, borderwidth=4, width=420, height=145)
borde_B1_label = Label(borde_B1, text="BOMBA 1", font=("calibri", 18), width=14).place(relx=.006, rely=0.06, anchor=W)
borde_B1.pack(anchor=NW)
borde_B1.place(x=10, y=5)

borde_CONTROL_MODE_B1 = Frame(FrameB1, relief=GROOVE, borderwidth=4, width=420, height=190)
borde_L_CONTROL_MODE_B1 = Label(borde_CONTROL_MODE_B1, text="CONTROL MODE", font=("calibri", 18), width=14).place(
    relx=.006, rely=0.06, anchor=W)
borde_CONTROL_MODE_B1.pack(anchor=NW)
borde_CONTROL_MODE_B1.place(x=10, y=155)

borde_CONTROL_MODE_B1 = Frame(FrameB1, relief=GROOVE, borderwidth=4, width=420, height=340)
borde_L_CONTROL_MODE_B1 = Label(borde_CONTROL_MODE_B1, text="CALIBRACION DE CAUDAL", font=("calibri", 18),
                                width=22).place(relx=.006, rely=0.06, anchor=W)
borde_CONTROL_MODE_B1.pack(anchor=NW)
borde_CONTROL_MODE_B1.place(x=10, y=350)

L_salida_PID_B1 = Label(FrameB1, text="Salida PID %", font=("calibri", 18), width=10)
L_salida_PID_B1.pack()
L_salida_PID_B1.place(x=15, y=50)

E_salida_PID_B1 = Entry(FrameB1, textvariable=OP_PID_ph, justify=CENTER, font=("calibri", 18), width=8)
E_salida_PID_B1.pack()
E_salida_PID_B1.place(x=200, y=50)

L_salida_PID_B1 = Label(FrameB1, text="SP PH", font=("calibri", 18), width=10)
L_salida_PID_B1.pack()
L_salida_PID_B1.place(x=15, y=100)

E_salida_PID_B1 = Entry(FrameB1, textvariable=SP_PID_ph, justify=CENTER, font=("calibri", 18), width=8)
E_salida_PID_B1.pack()
E_salida_PID_B1.place(x=200, y=100)

E_salida_PID_B1 = Entry(FrameB1, textvariable=MODO_ph, justify=CENTER, font=("calibri", 18), width=12)
E_salida_PID_B1.pack()
E_salida_PID_B1.place(x=120, y=200)

L_salida_PID_B1 = Label(FrameB1, text="PRODUCTO", font=("calibri", 18), width=10)
L_salida_PID_B1.pack()
L_salida_PID_B1.place(x=15, y=250)

combo_B1 = ttk.Combobox(FrameB1, font=("calibri", 18), width=11, justify=CENTER)
combo_B1.pack()
combo_B1.place(x=200, y=250)
combo_B1["values"] = ["ACIDO", "BASE", "NUTRIENTES"]
combo_B1.current(0)


def PRODUCTO_B1():
    print "signando _ B1"

    try:
        if combo_B1.get() == combo_B2.get() or combo_B1.get() == combo_B3.get():
            print "ERROR"
            asignar_producto.config(bg="red")
        else:
            asignar_producto.config(bg="green")
        if combo_B1.get() == "ACIDO":
            output_ph.setpin_aux(31)
        if combo_B1.get() == "BASE":
            output_ph.setpin(31)
        if combo_B1.get() == "NUTRIENTES":
            output_ph.setpin_auxx(31)

    except ValueError:
        asignar_producto.config(bg="red")


asignar_producto = Button(FrameB1, text='  ASIGNAR  ', command=PRODUCTO_B1, relief=GROOVE, font=("calibri", 18),
                          width=8)
asignar_producto.pack()
asignar_producto.place(x=100, y=295)

L_salida_PID_B1 = Label(FrameB1, text="PERIODO(seg)", font=("calibri", 18), width=14)
L_salida_PID_B1.pack()
L_salida_PID_B1.place(x=15, y=400)

combo_P_B1 = ttk.Combobox(FrameB1, font=("calibri", 18), width=8, justify=CENTER)
combo_P_B1.pack()
combo_P_B1.place(x=220, y=400)
combo_P_B1["values"] = ["10", "20", "30"]
combo_P_B1.current(0)

L_Caudal_B1 = Label(FrameB1, text="MEDICION", font=("calibri", 18), width=10)
L_Caudal_B1.pack()
L_Caudal_B1.place(x=15, y=450)

E_Caudal_B1 = Entry(FrameB1, textvariable=Caudal_B1, justify=CENTER, font=("calibri", 18), width=12)
E_Caudal_B1.pack()
E_Caudal_B1.place(x=210, y=450)


def B1_setup():
    print "CALIbrANDO"
    GPIO.output(31, True)  ## Enciendo
    time.sleep(10)
    GPIO.output(31, False)


Config_date = Button(FrameB1, text='CALIBRAR', command=B1_setup, relief=GROOVE, font=("calibri", 18), width=8)
Config_date.pack()
Config_date.place(x=50, y=500)


def ACEPTAR_CAL_B1():
    global Caudal_B1
    try:
        if Caudal_B1.get() == 0:
            print "ERROR"
            ACEPTAR_CAL_B1.config(bg="red")
        else:
            ACEPTAR_CAL_B1.config(bg="green")
    except ValueError:
        ACEPTAR_CAL_B1.config(bg="red")


ACEPTAR_CAL_B1 = Button(FrameB1, text='ACEPTAR', command=ACEPTAR_CAL_B1, relief=GROOVE, font=("calibri", 18), width=8)
ACEPTAR_CAL_B1.pack()
ACEPTAR_CAL_B1.place(x=50, y=600)

borde_B2 = Frame(FrameB2, relief=GROOVE, borderwidth=4, width=420, height=145)
borde_B2_label = Label(borde_B2, text="BOMBA 2", font=("calibri", 18), width=14).place(relx=.006, rely=0.06, anchor=W)
borde_B2.pack(anchor=NW)
borde_B2.place(x=5, y=5)

borde_CONTROL_MODE_B2 = Frame(FrameB2, relief=GROOVE, borderwidth=4, width=420, height=190)
borde_L_CONTROL_MODE_B2 = Label(borde_CONTROL_MODE_B2, text="CONTROL MODE", font=("calibri", 18), width=14).place(
    relx=.006, rely=0.06, anchor=W)
borde_CONTROL_MODE_B2.pack(anchor=NW)
borde_CONTROL_MODE_B2.place(x=5, y=155)

borde_CONTROL_MODE_B2 = Frame(FrameB2, relief=GROOVE, borderwidth=4, width=420, height=340)
borde_L_CONTROL_MODE_B2 = Label(borde_CONTROL_MODE_B2, text="CALIBRACION DE CAUDAL", font=("calibri", 18),
                                width=22).place(relx=.006, rely=0.06, anchor=W)
borde_CONTROL_MODE_B2.pack(anchor=NW)
borde_CONTROL_MODE_B2.place(x=5, y=350)

L_salida_PID_B2 = Label(FrameB2, text="Salida PID %", font=("calibri", 18), width=10)
L_salida_PID_B2.pack()
L_salida_PID_B2.place(x=15, y=50)

E_salida_PID_B2 = Entry(FrameB2, textvariable=OP_PID_ph, justify=CENTER, font=("calibri", 18), width=8)
E_salida_PID_B2.pack()
E_salida_PID_B2.place(x=200, y=50)

L_salida_PID_B2 = Label(FrameB2, text="SP PH", font=("calibri", 18), width=10)
L_salida_PID_B2.pack()
L_salida_PID_B2.place(x=15, y=100)

E_salida_PID_B2 = Entry(FrameB2, textvariable=SP_PID_ph, justify=CENTER, font=("calibri", 18), width=8)
E_salida_PID_B2.pack()
E_salida_PID_B2.place(x=200, y=100)

E_salida_PID_B2 = Entry(FrameB2, textvariable=MODO_ph, justify=CENTER, font=("calibri", 18), width=12)
E_salida_PID_B2.pack()
E_salida_PID_B2.place(x=120, y=200)

L_salida_PID_B2 = Label(FrameB2, text="PRODUCTO", font=("calibri", 18), width=10)
L_salida_PID_B2.pack()
L_salida_PID_B2.place(x=15, y=250)

combo_B2 = ttk.Combobox(FrameB2, font=("calibri", 18), width=11, justify=CENTER)
combo_B2.pack()
combo_B2.place(x=200, y=250)
combo_B2["values"] = ["ACIDO", "BASE", "NUTRIENTES"]
combo_B2.current(1)


def PRODUCTO_B2():
    print "signando _ B2"
    try:
        if combo_B2.get() == combo_B1.get() or combo_B2.get() == combo_B3.get():
            print "ERROR"
            asignar_producto_B2.config(bg="red")
        else:
            asignar_producto_B2.config(bg="green")
        if combo_B2.get() == "ACIDO":
            output_ph.setpin_aux(33)
        if combo_B2.get() == "BASE":
            output_ph.setpin(33)
        if combo_B2.get() == "NUTRIENTES":
            output_ph.setpin_auxx(33)
    except ValueError:
        asignar_producto_B2.config(bg="red")


asignar_producto_B2 = Button(FrameB2, text='  ASIGNAR  ', command=PRODUCTO_B2, relief=GROOVE, font=("calibri", 18),
                             width=8)
asignar_producto_B2.pack()
asignar_producto_B2.place(x=100, y=295)

L_salida_PID_B2 = Label(FrameB2, text="PERIODO(seg)", font=("calibri", 18), width=14)
L_salida_PID_B2.pack()
L_salida_PID_B2.place(x=15, y=400)

combo_P_B2 = ttk.Combobox(FrameB2, font=("calibri", 18), width=8, justify=CENTER)
combo_P_B2.pack()
combo_P_B2.place(x=220, y=400)
combo_P_B2["values"] = ["10", "20", "30"]
combo_P_B2.current(0)

L_Caudal_B2 = Label(FrameB2, text="MEDICION", font=("calibri", 18), width=10)
L_Caudal_B2.pack()
L_Caudal_B2.place(x=15, y=450)

E_Caudal_B2 = Entry(FrameB2, textvariable=Caudal_B2, justify=CENTER, font=("calibri", 18), width=12)
E_Caudal_B2.pack()
E_Caudal_B2.place(x=210, y=450)


def B2_setup():
    print "CALIbrANDO"
    GPIO.output(33, True)  ## Enciendo
    time.sleep(10)
    GPIO.output(33, False)


Config_date = Button(FrameB2, text='CALIBRAR', command=B2_setup, relief=GROOVE, font=("calibri", 18), width=8)
Config_date.pack()
Config_date.place(x=50, y=500)


def ACEPTAR_CAL_B2():
    global Caudal_B2
    try:
        if Caudal_B2.get() == 0:
            print "ERROR"
            ACEPTAR_CAL_B2.config(bg="red")
        else:
            ACEPTAR_CAL_B2.config(bg="green")
    except ValueError:
        ACEPTAR_CAL_B2.config(bg="red")
        print "ERROR NULL"


ACEPTAR_CAL_B2 = Button(FrameB2, text='ACEPTAR', command=ACEPTAR_CAL_B2, relief=GROOVE, font=("calibri", 18), width=8)
ACEPTAR_CAL_B2.pack()
ACEPTAR_CAL_B2.place(x=50, y=600)

borde_B3 = Frame(FrameB3, relief=GROOVE, borderwidth=4, width=420, height=145)
borde_B3_label = Label(borde_B3, text="BOMBA 3", font=("calibri", 18), width=14).place(relx=.006, rely=0.06, anchor=W)
borde_B3.pack(anchor=NW)
borde_B3.place(x=5, y=5)

borde_CONTROL_MODE_B3 = Frame(FrameB3, relief=GROOVE, borderwidth=4, width=420, height=190)
borde_L_CONTROL_MODE_B3 = Label(borde_CONTROL_MODE_B3, text="CONTROL MODE", font=("calibri", 18), width=14).place(
    relx=.006, rely=0.06, anchor=W)
borde_CONTROL_MODE_B3.pack(anchor=NW)
borde_CONTROL_MODE_B3.place(x=5, y=155)

borde_CONTROL_MODE_B3 = Frame(FrameB3, relief=GROOVE, borderwidth=4, width=420, height=340)
borde_L_CONTROL_MODE_B3 = Label(borde_CONTROL_MODE_B3, text="CALIBRACION DE CAUDAL", font=("calibri", 18),
                                width=22).place(relx=.006, rely=0.06, anchor=W)
borde_CONTROL_MODE_B3.pack(anchor=NW)
borde_CONTROL_MODE_B3.place(x=5, y=350)

L_salida_PID_B3 = Label(FrameB3, text="Salida PID %", font=("calibri", 18), width=10)
L_salida_PID_B3.pack()
L_salida_PID_B3.place(x=15, y=50)

E_salida_PID_B3 = Entry(FrameB3, textvariable=OP_PID_ph, justify=CENTER, font=("calibri", 18), width=8)
E_salida_PID_B3.pack()
E_salida_PID_B3.place(x=200, y=50)

L_salida_PID_B3 = Label(FrameB3, text="SP PH", font=("calibri", 18), width=10)
L_salida_PID_B3.pack()
L_salida_PID_B3.place(x=15, y=100)

E_salida_PID_B3 = Entry(FrameB3, textvariable=SP_PID_ph, justify=CENTER, font=("calibri", 18), width=8)
E_salida_PID_B3.pack()
E_salida_PID_B3.place(x=200, y=100)

E_salida_PID_B3 = Entry(FrameB3, textvariable=MODO_ph, justify=CENTER, font=("calibri", 18), width=12)
E_salida_PID_B3.pack()
E_salida_PID_B3.place(x=120, y=200)

L_salida_PID_B3 = Label(FrameB3, text="PRODUCTO", font=("calibri", 18), width=10)
L_salida_PID_B3.pack()
L_salida_PID_B3.place(x=15, y=250)

combo_B3 = ttk.Combobox(FrameB3, font=("calibri", 18), width=11, justify=CENTER)
combo_B3.pack()
combo_B3.place(x=200, y=250)
combo_B3["values"] = ["ACIDO", "BASE", "NUTRIENTES"]
combo_B3.current(2)


def PRODUCTO_B3():
    print "signando _ B3"
    try:
        if combo_B3.get() == combo_B1.get() or combo_B3.get() == combo_B2.get():
            print "ERROR"
            asignar_producto_B3.config(bg="red")
        else:
            asignar_producto_B3.config(bg="green")
        if combo_B3.get() == "ACIDO":
            output_ph.setpin_aux(35)
        if combo_B3.get() == "BASE":
            output_ph.setpin(35)
        if combo_B3.get() == "NUTRIENTES":
            output_ph.setpin_auxx(35)
    except ValueError:
        asignar_producto_B3.config(bg="red")


asignar_producto_B3 = Button(FrameB3, text='  ASIGNAR  ', command=PRODUCTO_B3, relief=GROOVE, font=("calibri", 18),
                             width=8)
asignar_producto_B3.pack()
asignar_producto_B3.place(x=100, y=295)

L_salida_PID_B3 = Label(FrameB3, text="PERIODO(seg)", font=("calibri", 18), width=14)
L_salida_PID_B3.pack()
L_salida_PID_B3.place(x=15, y=400)

combo_P_B3 = ttk.Combobox(FrameB3, font=("calibri", 18), width=8, justify=CENTER)
combo_P_B3.pack()
combo_P_B3.place(x=220, y=400)
combo_P_B3["values"] = ["10", "20", "30"]
combo_P_B3.current(0)

L_Caudal_B3 = Label(FrameB3, text="MEDICION", font=("calibri", 18), width=10)
L_Caudal_B3.pack()
L_Caudal_B3.place(x=15, y=450)

E_Caudal_B3 = Entry(FrameB3, textvariable=Caudal_B3, justify=CENTER, font=("calibri", 18), width=12)
E_Caudal_B3.pack()
E_Caudal_B3.place(x=210, y=450)


def B3_setup():
    print "CALIbrANDO B3"
    GPIO.output(35, True)  ## Enciendo
    time.sleep(10)
    GPIO.output(35, False)


Config_date = Button(FrameB3, text='CALIBRAR', command=B3_setup, relief=GROOVE, font=("calibri", 18), width=8)
Config_date.pack()
Config_date.place(x=50, y=500)


def ACEPTAR_CAL_B3():
    global Caudal_B3
    try:
        if Caudal_B3.get() == 0:
            print "ERROR"
            ACEPTAR_CAL_B3.config(bg="red")
        else:
            ACEPTAR_CAL_B3.config(bg="green")
    except ValueError:
        ACEPTAR_CAL_B3.config(bg="red")
        print "ERROR NULL"


ACEPTAR_CAL_B3 = Button(FrameB3, text='ACEPTAR', command=ACEPTAR_CAL_B3, relief=GROOVE, font=("calibri", 18), width=8)
ACEPTAR_CAL_B3.pack()
ACEPTAR_CAL_B3.place(x=50, y=600)

##################################FRAME 5 - SETUP ################################
######## 1 #set point PID--- scale TEMPERATURA

variable_label_CONFIG = Label(frame5, text="CONFIGURACION GENERAL", font=("calibri", 18), width=20)
variable_label_CONFIG.pack()
variable_label_CONFIG.place(x=0, y=0)

#####botones de configuracion de tiempo de muestreo

FrameT = Frame(frame5, highlightbackground="blue")
FrameT.pack()
FrameT.place(height=150, width=1300, x=0, y=0)
FrameT.config(background="white")
FrameD = Frame(frame5, relief=RAISED)
FrameD.pack()
FrameD.place(height=400, width=1300, x=0, y=100)

borde_date = Frame(FrameD, relief=GROOVE, borderwidth=4, width=1290, height=300)
borde_date_label = Label(borde_date, text="Configuracion Fecha y Hora", font=("calibri", 18), width=28).place(relx=.066,
                                                                                                              rely=0.04,
                                                                                                              anchor=W)
borde_date.pack(anchor=NW)
borde_date.place(x=5, y=5)

borde_muestreo = Frame(FrameT, relief=GROOVE, borderwidth=4, width=1290, height=90)
borde_muestreo_label = Label(borde_muestreo, text=" Frecuencia de Registro", font=("calibri", 18), width=22).place(
    relx=.066, rely=0.1, anchor=W)
borde_muestreo.pack(anchor=NW)
borde_muestreo.place(x=5, y=5)


def sel():
    print "Tiempo de muestreo: ", str(var.get())


variable = Label(FrameT, text="Tiempo de muestreo", font=("calibri", 18), width=18)
variable.pack()
variable.place(x=15, y=40)

b = Radiobutton(FrameT, text="1 min", variable=var, value=1, command=sel, font=("calibri", 18), width=6)
b.pack()
b.place(x=400, y=40)

b1 = Radiobutton(FrameT, text="5 min", variable=var, value=5, command=sel, font=("calibri", 18), width=6)
b1.pack()
b1.place(x=550, y=40)

b2 = Radiobutton(FrameT, text="10 min", variable=var, value=10, command=sel, font=("calibri", 18), width=6)
b2.pack()
b2.place(x=700, y=40)

b3 = Radiobutton(FrameT, text="15 min", variable=var, value=15, command=sel, font=("calibri", 18), width=6)
b3.pack()
b3.place(x=850, y=40)

b4 = Radiobutton(FrameT, text="30 min", variable=var, value=30, command=sel, font=("calibri", 18), width=6)
b4.pack()
b4.place(x=1000, y=40)

b5 = Radiobutton(FrameT, text="60 min", variable=var, value=60, command=sel, font=("calibri", 18), width=6)
b5.pack()
b5.place(x=1150, y=40)

date_config = Label(FrameD, text="  DD - MM - AAAA", font=("calibri", 18), width=14)
date_config.pack()
date_config.place(x=15, y=90)
time_config = Label(FrameD, text="  HH - MM   ", font=("calibri", 18), width=14)
time_config.pack()
time_config.place(x=15, y=140)

FECHA_dia = Entry(FrameD, textvariable=dia_date, justify=CENTER, state=DISABLED, font=("calibri", 18), width=8)
FECHA_dia.pack()
FECHA_dia.place(x=300, y=90)
FECHA_mes = Entry(FrameD, textvariable=mes_date, justify=CENTER, state=DISABLED, font=("calibri", 18), width=8)
FECHA_mes.pack()
FECHA_mes.place(x=450, y=90)
FECHA_anio = Entry(FrameD, textvariable=ano_date, justify=CENTER, state=DISABLED, font=("calibri", 18), width=8)
FECHA_anio.pack()
FECHA_anio.place(x=600, y=90)

HORA_hora = Entry(FrameD, textvariable=hora_date, justify=CENTER, state=DISABLED, font=("calibri", 18), width=8)
HORA_hora.pack()
HORA_hora.place(x=300, y=140)
HORA_minuto = Entry(FrameD, textvariable=minuto_date, justify=CENTER, state=DISABLED, font=("calibri", 18), width=8)
HORA_minuto.pack()
HORA_minuto.place(x=450, y=140)

ano_date.set(ano_date)
mes_date.set(mes_date)
dia_date.set(dia_date)
hora_date.set(hora_date)
minuto_date.set(minuto_date)


def update_date():
    global MODO_config
    MODO_config = 1
    print "recibiendo Fecha ..."


def config_date():
    global MODO_config
    global date_entry
    global ano_date
    global mes_date
    global dia_date
    global hora_date
    global minuto_date
    MODO_config = 2
    FECHA_anio.config(state=DISABLED, textvariable=ano_date)
    FECHA_mes.config(state=DISABLED, textvariable=mes_date)
    FECHA_dia.config(state=DISABLED, textvariable=dia_date)
    HORA_hora.config(state=DISABLED, textvariable=hora_date)
    HORA_minuto.config(state=DISABLED, textvariable=minuto_date)


def edit_date():
    global MODO_config
    global date_entry
    global ano_date_edit
    global mes_date_edit
    global dia_date_edit
    global hora_date_edit
    global minuto_date_edit
    global ano_date
    global mes_date
    global dia_date
    global hora_date
    global minuto_date
    
    ano_date_edit.set(ano_date.get())
    mes_date_edit.set(mes_date.get())
    dia_date_edit.set(dia_date.get())
    hora_date_edit.set(hora_date.get())
    minuto_date_edit.set(minuto_date.get())
    
    FECHA_anio.config(state=NORMAL, textvariable=ano_date_edit)
    FECHA_mes.config(state=NORMAL, textvariable=mes_date_edit)
    FECHA_dia.config(state=NORMAL, textvariable=dia_date_edit)
    HORA_hora.config(state=NORMAL, textvariable=hora_date_edit)
    HORA_minuto.config(state=NORMAL, textvariable=minuto_date_edit)


Config_date = Button(FrameD, text='Actualizar', command=update_date, relief=GROOVE, font=("calibri", 18), width=16)
Config_date.pack()
Config_date.place(x=900, y=85)

Config_date = Button(FrameD, text=' Modificar ', command=config_date, relief=GROOVE, font=("calibri", 18), width=16)
Config_date.pack()
Config_date.place(x=900, y=185)

edit_date = Button(FrameD, text='  Editar  ', command=edit_date, relief=GROOVE, font=("calibri", 18), width=16)
edit_date.pack()
edit_date.place(x=900, y=135)

##################################FRAME 6 - HISTORIAL ################################
# Graficar in Frame6 - grafica
# base de tiempo

FrameD = Frame(frame6, relief=RAISED)
FrameD.pack()
FrameD.place(height=400, width=1300, x=0, y=300)

borde_date = Frame(FrameD, relief=GROOVE, borderwidth=4, width=1290, height=300)
borde_date_label = Label(borde_date, text="GRAFICAR DATOS", font=("calibri", 18), width=28).place(relx=.066, rely=0.04,
                                                                                                  anchor=W)
borde_date.pack(anchor=NW)
borde_date.place(x=5, y=5)

# diseño de la ventana
fig = Figure(figsize=(3, 3))
a = fig.add_subplot(111)
a.set_title("Grafica General", fontsize=16)
a.set_ylabel("VARIABLES", fontsize=14)
a.set_xlabel("TIEMPO", fontsize=14)
b = a.twinx()
b.set_ylabel("RPM", color='r')
b.set_ylim(0, 1400)
b.tick_params('y', colors='r')

canvas2 = FigureCanvasTkAgg(fig, master=frame6)
canvas2.get_tk_widget().pack(side="top", fill="x")
combo = ttk.Combobox(FrameD, font=("calibri", 18), width=16)
combo.pack()
combo.place(x=170, y=50)
combo["values"] = list_hora + ["gaston"]
list_file = [""]
for f in os.listdir("registro"):
    
    if f.endswith(".csv"):
        list_file.append("registro/" + f)
    
combo["values"] = list_file
combo.current(0)
archivo_a_leer = combo.get()

combo_hora = ttk.Combobox(FrameD, font=("calibri", 18), width=16)
combo_hora.pack()
combo_hora.place(x=170, y=120)
combo_hora["values"] = list_hora + ["gaston"]
lines = []
try:
    graphData = open(archivo_a_leer, "r").read()
    lines = graphData.split("\n")
except:
    pass

dt = 1
t_base = np.arange(0, len(lines), dt)

xvalues = []
yvalues = []
zvalues = []
wvalues = []
horavalues = []
list_hora_aux = []

for line in lines:
    if len(line) > 1:
        hora, temp, ph, od, rpm = line.split(',')
        xvalues.append(temp)
        yvalues.append(ph)
        zvalues.append(od)
        wvalues.append(rpm)
        if hora == "00:00 " or hora == "01:00 " or hora == "02:00 " or hora == "03:00 " or hora == "04:00 " or hora == "05:00 " or hora == "06:00 " or hora == "07:00 " or hora == "08:00 " or hora == "09:00 " or hora == "10:00 " or hora == "11:00 " or hora == "12:00 " or hora == "13:00 " or hora == "14:00 " or hora == "15:00 " or hora == "16:00 " or hora == "17:00 " or hora == "18:00 " or hora == "19:00 " or hora == "20:00 " or hora == "21:00 " or hora == "22:00 " or hora == "23:00 ":
            horavalues.append(hora)
            list_hora_aux.append(hora)
        else:
            horavalues.append("")
t_base = np.arange(0, len(xvalues), dt)
a.set_xticks(t_base)
a.set_xticklabels(horavalues)
a.plot(t_base, xvalues)

a.plot(t_base, yvalues)
a.plot(t_base, zvalues)
a.plot(t_base, wvalues)

b.set_ylabel("RPM", color='r')
b.set_ylim(0, 1400)
a.set_ylabel("TEMP - PH - OD", fontsize=14)
b.plot(t_base, wvalues, 'r')

combo_hora["values"] = list_hora_aux
hora_a_leer = combo_hora.get()


# botones con distintas bases de tiempo - grafica
def Data_GRAFICAR():
    global dt
    global t_base
    global xvalues
    global yvalues
    global zvalues
    global wvalues
    global horavalues
    global combo
    print("refreshing data ...")
    archivo_a_leer = combo.get()
    try:
        graphData = open(archivo_a_leer, "r").read()
        lines = graphData.split("\n")
    except:
        pass
    
    dt = 1
    t_base = np.arange(0, len(lines), dt)
    xvalues = []
    yvalues = []
    zvalues = []
    wvalues = []
    horavalues = []
    list_hora_auxx = []

    for line in lines:
        if len(line) > 1:

            hora, temp, ph, od, rpm = line.split(',')
            xvalues.append(temp)
            yvalues.append(ph)
            zvalues.append(od)
            wvalues.append(rpm)
            if hora == "00:00 " or hora == "01:00 " or hora == "02:00 " or hora == "03:00 " or hora == "04:00 " or hora == "05:00 " or hora == "06:00 " or hora == "07:00 " or hora == "08:00 " or hora == "09:00 " or hora == "10:00 " or hora == "11:00 " or hora == "12:00 " or hora == "13:00 " or hora == "14:00 " or hora == "15:00 " or hora == "16:00 " or hora == "17:00 " or hora == "18:00 " or hora == "19:00 " or hora == "20:00 " or hora == "21:00 " or hora == "22:00 " or hora == "23:00 ":
                horavalues.append(hora)
                list_hora_auxx.append(hora)
            else:
                horavalues.append("")
    print len(t_base)
    print len(horavalues)
    if len(t_base) != len(horavalues):
        xvalues.append(temp)
        yvalues.append(ph)
        zvalues.append(od)
        wvalues.append(rpm)
        if hora == "00:00 " or hora == "01:00 " or hora == "02:00 " or hora == "03:00 " or hora == "04:00 " or hora == "05:00 " or hora == "06:00 " or hora == "07:00 " or hora == "08:00 " or hora == "09:00 " or hora == "10:00 " or hora == "11:00 " or hora == "12:00 " or hora == "13:00 " or hora == "14:00 " or hora == "15:00 " or hora == "16:00 " or hora == "17:00 " or hora == "18:00 " or hora == "19:00 " or hora == "20:00 " or hora == "21:00 " or hora == "22:00 " or hora == "23:00 ":
            horavalues.append(hora)
        else:
            horavalues.append("")

    a.clear()
    b.clear()
    a.set_xticks(t_base)
    a.set_xticklabels(horavalues)
    a.plot(t_base, xvalues, t_base, yvalues, t_base, zvalues, 'g')
    b.set_ylabel("RPM", color='r')
    b.set_ylim(0, 1400)
    a.set_ylabel("TEMP - PH - OD", fontsize=14)
    b.plot(t_base, wvalues, 'r')

    combo_hora["values"] = list_hora_auxx
    a.set_title(archivo_a_leer, fontsize=16)
    fig.canvas.draw()


def Data_GRAFICAR_1h():
    global dt
    global t_base
    global xvalues
    global yvalues
    global zvalues
    global wvalues
    global horavalues
    global combo_hora
    global combo
    global archivo_a_leer
    global hora_a_leer
    ban_hora_start = 0
    cont = 0
    hora_a_leer = combo_hora.get()
    print "mostrando hora:", hora_a_leer
    dt = 1
    t_base = np.arange(0, len(lines), dt)
    list_hora_auxx = []
    hora = []
    x = []
    y = []
    z = []
    w = []
    for line in horavalues:
        if ban_hora_start == 0:
            if line == hora_a_leer:
                x.append(xvalues[cont])
                y.append(yvalues[cont])
                z.append(zvalues[cont])
                w.append(wvalues[cont])
                hora.append(horavalues[cont])
                ban_hora_start = 1
        else:
            if line == '':
                x.append(xvalues[cont])
                y.append(yvalues[cont])
                z.append(zvalues[cont])
                w.append(wvalues[cont])
                hora.append(horavalues[cont])
            else:
                x.append(xvalues[cont])
                y.append(yvalues[cont])
                z.append(zvalues[cont])
                w.append(wvalues[cont])
                hora.append(horavalues[cont])
                break
        cont = cont + 1
    dt = 1
    t_base = np.arange(0, len(hora), dt)
    a.clear()
    b.clear()
    a.set_xticks(t_base)
    a.set_xticklabels(hora)
    a.plot(t_base, x, t_base, y, t_base, z, 'g')
    a.set_title("Grafica 1 Hora", fontsize=16)
    b.set_ylabel("RPM", color='r')
    b.set_ylim(0, 1400)
    a.set_ylabel("TEMP - PH - OD", fontsize=14)
    b.plot(t_base, w, 'r')
    fig.canvas.draw()


def Data_GRAFICAR_3h():
    global dt
    global t_base
    global xvalues
    global yvalues
    global zvalues
    global wvalues
    global horavalues
    global combo_hora
    global combo
    global archivo_a_leer
    global hora_a_leer
    ban_hora_start = 0
    cont = 0
    cont_aux = 0
    hora_a_leer = combo_hora.get()
    print "mostrando hora:", hora_a_leer
    dt = 1
    t_base = np.arange(0, len(lines), dt)
    list_hora_auxx = []
    hora = []
    x = []
    y = []
    z = []
    w = []
    for line in horavalues:
        if ban_hora_start == 0:
            if line == hora_a_leer:
                x.append(xvalues[cont])
                y.append(yvalues[cont])
                z.append(zvalues[cont])
                w.append(wvalues[cont])
                hora.append(horavalues[cont])
                ban_hora_start = 1
        else:
            if cont_aux < 3:
                if line == '':
                    x.append(xvalues[cont])
                    y.append(yvalues[cont])
                    z.append(zvalues[cont])
                    w.append(wvalues[cont])
                    hora.append(horavalues[cont])
                else:
                    x.append(xvalues[cont])
                    y.append(yvalues[cont])
                    z.append(zvalues[cont])
                    w.append(wvalues[cont])
                    hora.append(horavalues[cont])
                    cont_aux = cont_aux + 1
            else:
                break
        cont = cont + 1
    dt = 1
    t_base = np.arange(0, len(hora), dt)
    a.clear()
    b.clear()
    a.set_xticks(t_base)
    a.set_xticklabels(hora)
    a.plot(t_base, x)
    a.plot(t_base, y)
    a.plot(t_base, z)
    a.set_title("Grafica 3 Horas", fontsize=16)
    b.set_ylabel("RPM", color='r', fontsize=14)
    b.set_ylim(0, 1400)
    a.set_ylabel("TEMP - PH - OD", fontsize=14)
    b.plot(t_base, w, 'r')
    fig.canvas.draw()


def Data_GRAFICAR_6h():
    global dt
    global t_base
    global xvalues
    global yvalues
    global zvalues
    global wvalues
    global horavalues
    global combo_hora
    global combo
    global archivo_a_leer
    global hora_a_leer
    ban_hora_start = 0
    cont = 0
    cont_aux = 0
    hora_a_leer = combo_hora.get()
    print "mostrando hora:", hora_a_leer
    dt = 1
    t_base = np.arange(0, len(lines), dt)
    list_hora_auxx = []
    hora = []
    x = []
    y = []
    z = []
    w = []
    for line in horavalues:
        if ban_hora_start == 0:
            if line == hora_a_leer:
                x.append(xvalues[cont])
                y.append(yvalues[cont])
                z.append(zvalues[cont])
                w.append(wvalues[cont])
                hora.append(horavalues[cont])
                ban_hora_start = 1
        else:
            if cont_aux < 6:
                if line == '':
                    x.append(xvalues[cont])
                    y.append(yvalues[cont])
                    z.append(zvalues[cont])
                    w.append(wvalues[cont])
                    hora.append(horavalues[cont])
                else:
                    x.append(xvalues[cont])
                    y.append(yvalues[cont])
                    z.append(zvalues[cont])
                    w.append(wvalues[cont])
                    hora.append(horavalues[cont])
                    cont_aux = cont_aux + 1
            else:
                break
        cont = cont + 1
    dt = 1
    t_base = np.arange(0, len(hora), dt)
    a.clear()
    b.clear()
    a.set_xticks(t_base)
    a.set_xticklabels(hora)
    a.plot(t_base, x, t_base, y, t_base, z, 'g')
    a.set_title("Grafica 6 Horas", fontsize=16)
    b.set_ylabel("RPM", color='r', fontsize=14)
    b.set_ylim(0, 1400)
    a.set_ylabel("TEMP - PH - OD", fontsize=14)
    b.plot(t_base, w, 'r')
    fig.canvas.draw()


# Creacion de Botones para distintas bases de tiempo

B15 = Button(FrameD, text="GRAFICAR", command=Data_GRAFICAR, fg="Blue", font=("calibri", 18), width=14)
B15.pack()
B15.place(x=20, y=180)
B30 = Button(FrameD, text="1 HORA", command=Data_GRAFICAR_1h, fg="Blue", font=("calibri", 18), width=14)
B30.pack()
B30.place(x=270, y=180)
B1h = Button(FrameD, text="3 HORAS", command=Data_GRAFICAR_3h, fg="Blue", font=("calibri", 18), width=14)
B1h.pack()
B1h.place(x=520, y=180)
B6h = Button(FrameD, text="6 HORAS", command=Data_GRAFICAR_6h, fg="Blue", font=("calibri", 18), width=14)
B6h.pack()
B6h.place(x=770, y=180)
#############################################

GPIO.add_event_detect(PIN_IDLE, GPIO.RISING, callback=get_data)

# t=thread.start_new_thread(refreshPID,(T,))
# D=thread.start_new_thread(refreshData,(T,))

PWM_PH = thread.start_new_thread(refreshPWM_soft_ph, (T,))
fi = thread.start_new_thread(refreshfile, (T,))
pp.mainloop()
