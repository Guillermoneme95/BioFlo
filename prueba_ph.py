import math
def ph_T_correction(ph, T):
    k = 0.13671875
    k1 = 2.8
    k2 = 3.57
    
    R = 8.3144
    M = 2.303
    F = 96485
    #T = T + 273.15
    dT = T-25
    n = 1
    a = ph
    #ph = k  * 2.3 * ((R * T) / (n * F))  * math.log10(ph)
    ph = (M * R * dT *ph * n /F) * 2.8*3.57
    
    
    return round(ph, 2)

def test(ph):
    print("\n 0: " + str(ph_T_correction(ph, 0))+
    #"\n 10: "+ str( ph_T_correction(ph, 10))+
    "\n 25: "+ str( ph_T_correction(ph, 25))+
    "\n 100: "+ str( ph_T_correction(ph, 60)))

test(4)
test(7)
test(10)

