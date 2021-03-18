# -*- coding: utf-8 -*-


from __future__ import division
import serial
import time
import RPi.GPIO as GPIO
import datetime
from collections import namedtuple
import time
import math
import binascii
import sys

"""
Glosary
T: Temperatura
od: Oxígeno Disuelto
"""

# port = "/dev/ttyAMA0"  # Raspberry Pi 2
# port = "/dev/ttyS0"    # Raspberry Pi 3

BOUNDS = dict(t=(0, 100),
                            ph=(0, 100),
                            od=(0, 100),
                            rpm=(0, 1500)
                            )

MODES = {
                    "status": "a",
                     "datetime": "b",
                     "set_datetime": "c",
                     "T_slope": "d",
                     "set_T_slope":"e",
                     "ph_slope":"f",
                     "set_ph_slope":"g",
                     "od_slope":"h",
                     "set_od_slope":"i",
                     "rpm_slope":"j",
                     "set_rpm_slope":"k"}

DEFAULT_RESET_MSG = "Reiniciando Módulo de Sensado: "

Status = namedtuple("Status", ["T", "ph", "od", "rpm", "datetime"])             

def ph_T_correction(ph, T):
    k1 = 2.8
    k2 = 3.57
    R = 8.3144
    M = 2.303
    F = 96485
    dT = T - 25
    n = 1
    ph = ph - (M * R * dT * ph * n /F) * k1 * k2
    return round(ph, 1)

class ModuloSensado(object):
    def __init__(self, pin_reset=16, pin_idle=13, port="/dev/ttyAMA0", 
                        baudrate=9600, timeout=0.5):
        GPIO.setmode(GPIO.BOARD)
        GPIO.setwarnings(False)
        GPIO.setup(pin_reset, GPIO.OUT)
        GPIO.setup(pin_idle, GPIO.IN)    
        
        self.pin_reset = pin_reset
        self.pin_idle = pin_idle
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout

        self.error_counter = 0
        self.time_last_sensing = None
        self.last_senting = None
        self.serialport = serial.Serial(port, baudrate=baudrate, timeout=timeout)
        self.reset("Inicializando sistema")

    def reset(self, msg="") :
        GPIO.output(self.pin_reset, True)
        print(DEFAULT_RESET_MSG + str(msg))
        time.sleep(1)
        GPIO.output(self.pin_reset, False)
        time.sleep(1)
        self.error_counter = 0

    def synchronize_time(self):
        pass

    def _wait_idle(self):
        i_idle = 0
        while True:
            if GPIO.input(self.pin_idle):
                break
            elif i_idle > 6000:
                self.reset("Pin de módulo desocupado no se encendió por más de 5 segundos")
            else:
                time.sleep(0.05)
        return True

    def _check_idle(self):
        return GPIO.input(self.pin_idle)
    
    def _read_serial(self, mode, idle_mode=None,
                                 data_formatter=None, data_checker=None, retries=3):
        if idle_mode:
            if not idle_mode():
                return False
        for retry in range(retries):
            if self.error_counter > 12:
                self.reset("Límite de errores de comunicación superado.")
                pass
            try:
                self.serialport.reset_input_buffer()
                self.serialport.reset_output_buffer()
                self.serialport.write(MODES[mode])
                #data = self.serialport.readline()
                data = self.serialport.read_until('\x0013')

                if (len(data) == 0):
                    self.error_counter += 1
                    print("No se recibieron datos. Mode: %s"%mode)
                    continue
                if data_formatter:
                        data = data_formatter(data)
                if data_checker:
                    if data_checker(data):
                        return data
                else:
                    return data
            except serial.SerialTimeoutException:
                self.error_counter += 1
                print("Tiempo de espera superado. Mode: %s"%mode)
            except:
                self.error_counter += 1
                print("Datos erroneos. Mode: %s"%mode)
                #print("----\n", sysexc_info)
        else: 
            return False

    def _status_formatter(self, data):
        datetime_ = datetime.datetime.now()
        try:
            list_concatenados = [int(binascii.hexlify(data)[i*4: i*4+4], base=16) for i in xrange(4)]
        except:
            raise ValueError("Error Convirtiendo: ", [binascii.hexlify(data)[i*4: i*4+4] for i in xrange(4)])
            
        
        ph = ph_T_correction(ph=list_concatenados[1]/10, T=list_concatenados[0]/10) 
        # Descripción Status("T" "ph", "od", "rpm", "datetime") 
        return Status(list_concatenados[0]/10, ph, 
                                list_concatenados[2]/10, list_concatenados[3],
                                 datetime_)
    def _slope_formatter(self, data):
        return [int(binascii.hexlify(data)[i*4: i*4+4], base=16) for i in xrange(2)]
    def _datetime_formatter(self, data):
        date_list = [int(data[i*2:i*2+2]) for i in range(5)]
        
        return datetime.datetime(2000+date_list[2], date_list[1], date_list[0], date_list[3], date_list[4])
        
    def _status_checker(self, data):
        return True
    
    def get_data(self):
        data = self._read_serial(mode="status", idle_mode=self._wait_idle,
                                                 data_formatter=self._status_formatter, 
                                                 data_checker=self._status_checker,
                                                 retries=3)
        return data

    def get_datetime(self):
        data = self._read_serial(mode="datetime", idle_mode=self._wait_idle,
                                                 data_formatter=self._datetime_formatter, 
                                                 retries=10)
        return data
        
    def get_slope(self, var):
            data = self._read_serial(mode="%s_slope"%var, idle_mode=self._wait_idle,
                                                     data_formatter=self._slope_formatter, 
                                                     retries=3)
            return data

    def set_datetime(self, year, month, day, hour, minute):
        try:
            datetime_ = datetime.datetime(int(year), int(month), int(day), int(hour), int(minute))
            
            year = str(year)[-2:]
            datetime_str = "%.02d%.02d%.02d%.02d%.02d"%(int(day),int(month),int(year),
                                                                                             int(hour),int(minute))
            self.serialport.reset_input_buffer()
            self.serialport.reset_output_buffer()
            self.serialport.write(MODES["set_datetime"])
        except ValueError:
            print("Fecha Incorrecta")
            return False

        return self.serialport.write(datetime_str)
    def set_slope(self, var, ref_H, ref_L, vi_H,vi_L, calH, calO):
            try:
                cer = vi_L - ref_L + calO#* 10
                #if var != "rpm":
                #   cer = cer * 10
                print("Cero calibrado: " + str(cer))
                pend = (ref_H / vi_H) * calH + cer # NUEVA PENDIENTE
                print("Pendiente calibrada: " + str(pend))
                entero_pend = int(pend)
                entero_cer = int(cer)  # NUEVO CERO
                cociente = entero_pend // 256
                resto = entero_pend % 256
                cer_cociente = entero_cer // 256
                cer_resto = entero_cer % 256
                cadena = chr(cociente) + chr(resto) + chr(cer_cociente) + chr(cer_resto)
                #print "pendiente %s, cero %s"%(pend, cer)
                #print "modo set_%s_slope: cociente%s, resto%s, cero%s, cero_cero%s "%(var, cociente, resto, cer_cociente, cer_resto)
                self.serialport.reset_input_buffer()
                self.serialport.reset_output_buffer()
                self.serialport.write(MODES["set_%s_slope"%var])
                return self.serialport.write(cadena)
            except:
                print("Ocurrió un error")
                return False
    ## NO
    def set_slope_(self, var, ref_H, ref_L, vi_H,vi_L, calH):
            try:
                print"Recibiendo var %s H%s L%s viH%s viL%s Ch%s"%(var, ref_H, ref_L, vi_H, vi_L, calH)
                #pend = (ref_H / vi_H) * calH  # NUEVA PENDIENTE
                pend = (ref_H - ref_L) / ((vi_L - ref_L) * calH)
                entero_pend = int(round(pend))
                cer = vi_L - ref_L
                if var == "rpm":
                    cer = cer * 10
                entero_cer = int(round(cer))  # NUEVO CERO
                cociente = entero_pend // 256
                resto = entero_pend % 256
                cer_cociente = entero_cer // 256
                cer_resto = entero_cer % 256
                cadena = chr(cociente) + chr(resto) + chr(cer_cociente) + chr(cer_resto)
                print "pendiente %s, cero %s"%(pend, cer)
                print "modo set_%s_slope: cociente%s, resto%s, cero%s, cero_cero%s "%(var, cociente, resto, 0, cer_cer)
                self.serialport.reset_input_buffer()
                self.serialport.reset_output_buffer()
                self.serialport.write(MODES["set_%s_slope"%var])
                return self.serialport.write(cadena)
            except:
                print("Ocurrió un error")
                return False
    #No
    def set_slope2(self, var, ref_H, ref_L, vi_H,vi_L, calH):
            try:
                print"Recibiendo var %s H%s L%s viH%s viL%s Ch%s"%(var, ref_H, ref_L, vi_H, vi_L, calH)
                #pend = (ref_H / vi_H) * calH  # NUEVA PENDIENTE
                #pend = (ref_H - ref_L) / ((vi_L - ref_L) * calH)
                v_max = vi_H / calH
                v_min = vi_L / calH  
                pend = ref_H
                entero_pend = int(pend)
                #cer = vi_L - ref_L
                cer = vi_L
                if var == "rpm":
                    cer = cer * 10
                entero_cer = int(cer)  # NUEVO CERO
                cociente = entero_pend // 256
                resto = entero_pend % 256
                cer_cer = entero_cer % 256
                cadena = chr(cociente) + chr(resto) + chr(0) + chr(cer_cer)
                print "pendiente %s, cero %s"%(pend, cer)

                print "modo set_%s_slope: cociente%s, resto%s, cero%s, cero_cero%s "%(var, cociente, resto, 0, cer_cer)
                self.serialport.reset_input_buffer()
                self.serialport.reset_output_buffer()
                self.serialport.write(MODES["set_%s_slope"%var])
                return self.serialport.write(cadena)
            except:
                print("Ocurrió un error")
                return False
    
    def hard_set_rpm(self, pend, cer):
        entero_pend = int(pend)
        entero_cer = int(cer)  # NUEVO CERO
        cociente = entero_pend // 256
        resto = entero_pend % 256
        cer_cociente = entero_cer // 256
        cer_resto = entero_cer % 256
        cadena = chr(cociente) + chr(resto) + chr(cer_cociente) + chr(cer_resto)
        self.serialport.reset_input_buffer()
        self.serialport.reset_output_buffer()
        self.serialport.write("k")
        return self.serialport.write(cadena)

    def hard_set_od(self, pend, cer):
        entero_pend = int(pend)
        entero_cer = int(cer)  # NUEVO CERO
        cociente = entero_pend // 256
        resto = entero_pend % 256
        cer_cociente = entero_cer // 256
        cer_resto = entero_cer % 256
        cadena = chr(cociente) + chr(resto) + chr(cer_cociente) + chr(cer_resto)
        self.serialport.reset_input_buffer()
        self.serialport.reset_output_buffer()
        self.serialport.write("i")
        return self.serialport.write(cadena)
    
    def reset_slopes(self):
        #default_calibrations =   dict(T=(800, 0), ph=(140, 0), od=(3125,0), rpm=(1500, 0) )
        default_calibrations =   dict(T=(799, 3), ph=(140, 0), od=(3125,0), rpm=(1228, 0) )  
        # Temperatura
        self.serialport.write('e')
        print "modo E - reset Temperatura" 
        m = 690#785
        o = 35   #6.4
        cociente = m // 256
        resto = m % 256
        num =  o % 256
        cadena = chr(cociente) + chr(resto) + chr(0) + chr(num)
        
        print (m, o)
        self.serialport.write(cadena)
        time.sleep(2)

        self.serialport.reset_input_buffer()
        self.serialport.reset_output_buffer()
        self.serialport.write('g')
        print "modo G - reset ph"
        m = 109#149.53
        o = 1#-0.9
        cociente = m // 256
        resto = m % 256
        num =  o % 256
        cadena = chr(cociente) + chr(resto) + chr(0) + chr(num)
        print (m, o)
        self.serialport.write(cadena)
        time.sleep(2)

        self.serialport.reset_input_buffer()
        self.serialport.reset_output_buffer()
        self.serialport.write('i')
        print "modo I - reset OD"
        m = 1731
        o = 0
        cociente = m // 256
        resto = m % 256
        cadena = chr(cociente) + chr(resto) + chr(0) + chr(0)
        print (m, o)
        self.serialport.write(cadena)
        time.sleep(2)

        self.serialport.reset_input_buffer()
        self.serialport.reset_output_buffer()
        self.serialport.write('k')
        print "modo K - reset RPM"
        #m = 1500
        m = 1228
        o = 0
        cociente = m // 256
        resto = m % 256
        num =  o % 256  # agredado para corregir offset motor
        cadena = chr(cociente) + chr(resto) + chr(0) + chr(num)
        print (m, o)
        self.serialport.write(cadena)

if __name__ == '__main__':
    s = ModuloSensado()
    #print(s.get_data())
    #print(s.get_datetime())
    #s.set_datetime("2018", "1", 2, 11,3)
    #serialport = serial.Serial("/dev/ttyAMA0", baudrate=9600, timeout=0.5)
    
