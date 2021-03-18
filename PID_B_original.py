class PID:
    def __init__(self, P=2.0, I=0.566, D=0.14,tipo="NORMAL", Derivator=0.0, Integrator=0.0, Integrator_max=1500.0, Integrator_min=-1500.0, T=1.0):
        self.Kp=P
        self.Ki=I
        self.Kd=D
        self.tipo=tipo
        self.Derivator=Derivator
        self.Integrator=Integrator
        self.Integrator_max=Integrator_max
        self.Integrator_min=Integrator_min
        self.T=T
        self.set_point=0.0
        self.error=0.0
        self.real_PID = 0
        if tipo=="TEMP":
            self.Integrator = 101*2.5

    def update(self,current_value):
        self.error = self.set_point - current_value
        self.P_value =  self.error
        self.D_value = (self.Kd/self.T) * (current_value - self.Derivator)
        self.Derivator = current_value
        self.Integrator = self.Integrator + self.error
        if self.Integrator > self.Integrator_max:
	        self.Integrator = self.Integrator_max
        elif self.Integrator < self.Integrator_min:
	        self.Integrator = self.Integrator_min

        self.I_value = self.Integrator * (self.T/self.Ki)
        
        PID = (self.Kp*(self.P_value + self.I_value - self.D_value))/5
        if self.tipo=="TEMP_":
                print("%s - P:%s I:%s D:%s PID:%s e:%s" % (
                    self.tipo, self.Kp * self.P_value / 5, self.Kp * self.I_value / 5, self.Kp * self.D_value / 5, PID, self.error))
        self.real_PID = PID
        if self.tipo=="NORMAL":
                if PID > 100:
                        PID = 100
                elif PID < 0:
                        PID = 0
                return PID
        elif self.tipo=="DOBLE":
                if PID > 100:
                        PID = 100
                elif PID < -100:
                        PID = -100
                return PID
        if self.tipo=="TEMP":
                if PID > 100:
                        PID = 100
                elif PID < 0:
                        PID = -1
                return PID
                

    def setPoint(self,set_point):		
	    self.set_point = set_point		

    def setIntegrator(self, Integrator):
	    self.Integrator = Integrator

    def setDerivator(self, Derivator):
	    self.Derivator = Derivator

    def setKp(self,P):
	    self.Kp=P

    def setKi(self,I):
	    self.Ki=I

    def setKd(self,D):
	    self.Kd=D
    def settipo(self,tipo):
	    self.tipo=tipo
	
    def setT(self,T):
	    self.T=T

    def getPoint(self):
	    return self.set_point

    def getError(self):
	    return self.error

    def getIntegrator(self):
	    return self.Integrator

    def getDerivator(self):
	    return self.Derivator

    def getT(self):
	    return self.T
