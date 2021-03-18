import RPi.GPIO as GPIO
import time
class PWM:
	def __init__(self, pin =33,T=1.0,tipo="NORMAL",pin_aux=35,pin_auxx=31): # T=periodo seg,
            self.pin=pin               
            self.T=T
            self.tipo=tipo
            self.pin_aux=pin_aux
            self.pin_auxx=pin_auxx
	    GPIO.setmode(GPIO.BOARD)
            GPIO.setwarnings(False)
            GPIO.setup(self.pin, GPIO.OUT)
            GPIO.setup(self.pin_aux, GPIO.OUT)
            

	def SetPWM(self,duty,state,signo):
            self.duty=duty	   
	    self.state=state
	    self.signo=signo ## signo=0 significa positivo y signo==1 negativo
            if self.state==1:
                    if self.tipo=="NORMAL":
                            if self.duty<=100 and self.duty>0:
                                    GPIO.output(self.pin, True) ## Enciendo                 
                                    time.sleep((self.duty/100.0)*self.T) ## Esperamos 
                                    GPIO.output(self.pin, False) ## Apago              
                                    time.sleep(self.T-(self.duty/100.0)*self.T)
                            else:
                                    time.sleep(self.T)
                    elif self.tipo=="DOBLE":
                            if self.signo==0:
                                    if self.duty<=100 and self.duty>0:
                                        GPIO.output(self.pin, True) ## Enciendo                 
                                        time.sleep((self.duty/100.0)*self.T) ## Esperamos 
                                        GPIO.output(self.pin, False) ## Apago              
                                        time.sleep(self.T-(self.duty/100.0)*self.T)
                                    else:
                                        time.sleep(self.T)
                            elif self.signo==1:
                                    if self.duty<=100 and self.duty>0:
                                        GPIO.output(self.pin_aux, True) ## Enciendo                 
                                        time.sleep((self.duty/100.0)*self.T) ## Esperamos 
                                        GPIO.output(self.pin_aux, False) ## Apago              
                                        time.sleep(self.T-(self.duty/100.0)*self.T)
                                    else:
                                        time.sleep(self.T)
        def setpin(self,P):
                self.pin=P
                
        def setpin_aux(self,P_aux):
                self.pin_aux=P_aux
                
        def setpin_auxx(self,P_auxx):
                self.pin_auxx=P_auxx
		

