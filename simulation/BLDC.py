#
# initial model comes from https://webfiles.portal.chalmers.se/et/MSc/BaldurssonStefanMSc.pdf

import numpy as np
import matplotlib.pyplot as plt


class BLDC:
    def __init__(self, R, L, J, Kt, Ke, Kf, pole_pairs):
        self.R = R  # Resistance (Ω)
        self.L = L  # Inductance (H)
        self.J = J  # Inertia (kg*m^2)
        self.Kt = Kt  # Torque constant (N*m/A)
        self.Ke = Ke  # Back-EMF constant (V/(rad/s))
        self.Kf = Kf  # Friction constant (N*m*s)
        self.pole_pairs = pole_pairs  # Number of poles
        self.dt = dt
        self.omega = 0
        self.theta = 0  # 􏰕a{qVz􏰟lfc􏰄epnjg􏰤nve
        self.I = np.array([0.0, 0.0, 0.0])

    def trapezoidal_waveform(self, offset=0):
        self.theta = self.theta % (2*np.pi)
        theta_e = self.theta * self.pole_pairs  # electrical angle

        theta_e += offset
        theta_e = theta_e % (2*np.pi)
        if theta_e >= 0 and theta_e <= np.pi/3:
            return 1
        elif theta_e >= np.pi/3 and theta_e <= np.pi:
            return 1-6/np.pi*(theta_e-2*np.pi/3)
        elif theta_e >= np.pi and theta_e <= 5*np.pi/3:
            return -1
        elif theta_e >= 5*np.pi/3 and theta_e <= 2*np.pi:
            return -1+6/np.pi*(theta_e-5*np.pi/3)

    def update_state(self, V, TL, dt):
        ia, ib, ic = self.I
        va, vb, vc = V
        ea = self.Ke/2*self.omega*self.trapezoidal_waveform(0)
        eb = self.Ke/2*self.omega*self.trapezoidal_waveform(-2*np.pi/3)
        ec = self.Ke/2*self.omega*self.trapezoidal_waveform(-4*np.pi/3)

        Te = self.Kt/2*(ia*self.trapezoidal_waveform(0) + ib *
                        self.trapezoidal_waveform(-2*np.pi/3) + ic*self.trapezoidal_waveform(-4*np.pi/3))
        vab = va-vb
        eab = ea-eb
        vbc = vb-vc
        ebc = eb-ec
        dia_dt = -self.R/self.L*ia+2/3/self.L*(vab-eab)+1/3/self.L*(vbc-ebc)
        dib_dt = -self.R/self.L*ib-1/3/self.L*(vab-eab)+1/3/self.L*(vbc-ebc)
        domega_dt = -self.Kf/self.J*self.omega+(Te-TL)/self.J
        self.I += np.array([dia_dt, dib_dt, -dia_dt-dib_dt])*dt
        self.theta += self.omega*dt+0.5*domega_dt*dt*dt
        self.omega += domega_dt*dt

        return self.I, self.theta, self.omega


if __name__ == "__main__":
    # Set the motor parameters
    R = 0.1
    L = 0.5
    J = 0.01

    Kt = 0.1
    Ke = 0.1
    Kf = 0.01
    pole_pairs = 6
    dt = 0.01
    T = 100
    # Create an instance of the BLDC class
    motor = BLDC(R, L, J, Kt, Ke, Kf, pole_pairs)

    # Set the initial voltages
    V = np.array([0, 0, 0])
    times = []
    thetas = []
    omegas = []
    # Run the simulation for 1000 timesteps
    for i in range(int(T/dt)):
        time = i*dt
        vA = np.sin(2 * np.pi * 50 * time)
        vB = np.sin(2 * np.pi * 50 * time + 2 * np.pi / 3)
        vC = np.sin(2 * np.pi * 50 * time + 4 * np.pi / 3)
        V = np.array([vA, vB, vC])
        I, theta, omega = motor.update_state(V, 0, dt)
        print(f"Timestep {i}: I = {I}, theta = {theta}")
        times.append(time)
        thetas.append(theta)
        omegas.append(omega)
    # plt.plot(times, thetas)
    plt.plot(times, omegas)
    plt.show()
