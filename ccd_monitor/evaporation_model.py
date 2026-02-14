
import numpy as np

class AcetoneEvaporationModel:
    def __init__(self):
        # Acetone Properties
        self.Mw = 58.08  # g/mol
        self.R = 8.314   # J/(mol K)
        self.T_boil = 329.2 # K (56C)
        self.LatentHeat = 29100 # J/mol (at boiling) or ~500 J/g = 29000 J/mol
        self.Cp_liq = 125 # J/(mol K) roughly 2.15 J/gK * 58
        self.Rho = 0.784 # g/cm3
        
        # Environmental / Setup Defaults
        self.T_amb = 298.15 # 25 C
        self.P_atm = 101325 # Pa
        
        # Model Parameters (To be fitted/tuned)
        self.k_mass = 1e-4  # Mass transfer coeff factor (roughly)
        self.h_heat = 10.0   # Heat transfer coeff (W/m2K) - natural convection
        self.Area = 1e-4 # Surface area (cm2 -> m2)? 1cm2 = 1e-4 m2
        self.Vol0 = 2e-6 # 2 mL = 2e-6 m3
        
    def antoine_vapor_pressure(self, T_kelvin):
        # Acetone Antoine Parameters (Bar)
        # log10(P) = A - B / (T + C)
        A = 4.42448
        B = 1312.253
        C = -32.445
        T_iso = T_kelvin
        if T_iso < 0: T_iso = 273 # sanity
        
        logP = A - B / (T_iso + C)
        P_bar = 10**logP
        P_pa = P_bar * 100000
        return P_pa

    def simulate(self, t_minutes, k_evap_factor=1.0):
        # Initial State
        m_moles = (self.Vol0 * self.Rho * 1e6) / self.Mw # g / g/mol = mol
        T = self.T_amb
        
        dt = 10.0 # seconds
        t_total = t_minutes * 60
        steps = int(t_total / dt)
        
        correction_factor = 1.0
        
        current_m = m_moles
        current_T = T
        
        for _ in range(steps):
             # 1. Vapor Pressure
             P_vap = self.antoine_vapor_pressure(current_T)
             
             # 2. Evaporation Rate (moles/s)
             # Rate = k * Area * (P_vap - P_inf)
             # We bundle k*Area into 'K_eff'
             # k_evap_factor scales this
             dm_dt = - self.k_mass * k_evap_factor * (P_vap / self.P_atm) 
             
             # 3. Heat Balance
             # m Cp dT/dt = h A (Tamb - T) + dm/dt * L
             Q_in = self.h_heat * self.Area * (self.T_amb - current_T)
             Q_evap = dm_dt * self.LatentHeat # dm_dt is negative, so this is Negative heat (Cooling) -> wait.
             # dm_dt is negative (mass loss). 
             # Latent heat is energy required to vaporize.
             # Heat loss = Rate * L. Rate is positive for evap.
             # So if dm_dt is negative, EvapRate = -dm_dt
             # Term is - (-dm_dt * L) = dm_dt * L. (Negative term).
             
             dT_dt = (Q_in + dm_dt * self.LatentHeat) / (current_m * self.Cp_liq)
             
             # Update
             current_m += dm_dt * dt
             current_T += dT_dt * dt
             
             if current_m <= 0:
                 current_m = 1e-9 # avoid zero div
                 break
                 
        # Concentration Factor = V0 / V_final = m0 / m_final
        initial_m = (self.Vol0 * self.Rho * 1e6) / self.Mw
        correction_factor = initial_m / current_m
        
        return correction_factor

if __name__ == "__main__":
    # Test
    model = AcetoneEvaporationModel()
    print("Time(min) | Correction")
    for t in [0, 5, 10, 30, 60]:
        cf = model.simulate(t, k_evap_factor=1.0)
        print(f"{t:4d}      | {cf:.4f}")
