#!/usr/bin/env python

# The electrical side of the kicker is a series RLC circuit.
# The current around the loop is measured directly.
#
# The measured voltage, however, does not map to the voltage across any ideal component.
# The capacitor and inductor each have an internal resistance.
# Voltage is measured between these two unknown resistances.

import scipy.integrate
import scipy.optimize
from numpy import *
from matplotlib.pyplot import *

# Channel 1 (row 0): current, 100A/V
# Channel 2 (row 1): voltage, 1V/V
# 200ksamples/s

assert len(sys.argv) == 2

# Find the beginning of discharge
data = loadtxt(sys.argv[1])
for start in range(size(data, 1)):
    if data[1, start] > 10:
        break

# Number of post-trigger (useful) samples
num_samples = size(data, 1) - start

# Sample rate, fixed by scope configuration
sample_rate = 200e3

# Current sense resistor
rsense = 0.010

# Time of each sample
tt = arange(0, num_samples) / sample_rate
# print arange(0, num_samples),num_samples

# Time of last sample
tmax = (num_samples - 1) / sample_rate

# Current out of the capacitor
i_rlc = array(data[0, start:] / rsense)

# Voltage across the capacitor and its internal resistance
v = array(data[1, start:])

# Maximum current
imax = max(i_rlc)

# Index of maximum current
imax_pos = list(i_rlc).index(imax)


# Find the parameters of the damped oscillation with least squares estimation.
# This depends only on the measured current, so it will not be affected by the unknown
# resistances in the capacitor and inductor.
def residual_i(X):
    (wd, a, s) = X
    fit_i = exp(-a * tt) * sin(wd * tt) * s
    return i_rlc - fit_i


((wd, a, iscale), success) = scipy.optimize.leastsq(residual_i, [100, 100,
                                                                 1000])
print wd, a, iscale, success
print residual_i
assert success

# Best-fit current
fit_i = exp(-a * tt) * sin(wd * tt) * iscale

fit_imax = max(fit_i)

# Index of largest best-fit current.
# This is the position where the inductor starts discharging.
fit_imax_pos = list(fit_i).index(fit_imax)

fit_imax_time = fit_imax_pos / sample_rate


# Best-fit current evaluated at an arbitrary time.
# Use this for integration.
def eval_fit_i(t):
    return exp(-a * t) * sin(wd * t) * iscale

# Maximum voltage
vmax = max(v)

# Calculate capacitance by assuming the voltage decays to zero over a long time.
# Integrate the current over time to find the capacitance required to change from vmax to 0 volts.
# In theory, the total delivered energy would work, but there is significant integration error.
#
# vmax - scipy.integrate.quad(eval_fit_i, 0, 1) / c = 0
# vmax * c = scipy.integrate.quad(eval_fit_i, 0, 1)
c = scipy.integrate.quad(eval_fit_i, 0, 1)[0] / vmax

# Find the total charge that has left the capacitor at each time.
# Use the best-fit current for more accurate integration.
charge = array([scipy.integrate.quad(eval_fit_i, 0, t)[0] for t in tt])

#charge = array([scipy.integrate.trapz(fit_i[:n], dx=1/sample_rate) for n in range(num_samples)])


def residual_c(X):
    #(vmax, c, rc) = X
    rc = X
    return v - ((vmax - charge / c) - i_rlc * rc)

# It sounds like a good idea to fit Vmax, C, and Rc together, but this results in larger total energy errors
# than if we use Vmax and C as found above and only use least squares to estimate Rc.
#((vmax, c, rc), success) = scipy.optimize.leastsq(residual_c, [vmax, 2*2400e-6, 0.1])

# Find the capacitor's ESR (Rc).
((rc), success) = scipy.optimize.leastsq(residual_c, [0.1])
assert success

# Best-fit voltage across ideal capacitance
fit_vc = vmax - charge / c

# Best-fit voltage across measurement point
fit_v = fit_vc - fit_i * rc

# Find dampled-sinusoid parameters w0 and z
# a = w0 * z
# w0 = a / z
# wd = w0 * sqrt(1 - z**2)
# wd = a / z * sqrt(1 - z**2)
# wd / a = sqrt(1 - z**2) / z
z = sqrt(1 / ((wd / a)**2 + 1))
w0 = wd / sqrt(1 - z**2)

# w0 = 1/sqrt(L*c)
L = 1.0 / (c * w0**2)

# Total resistance (Rc + Rl)
# z = R / (2 * L * w0)
r = z * (2 * L * w0)

# Inductor resistance
rl = r - rc - rsense

# Power out of the capacitor+Rc according to curve fit
fit_p = fit_v * fit_i

# Power dissipated in the total resistance
pr = i_rlc**2 * r
fit_pr = fit_i**2 * r

# Power leaving the capacitor (excluding loss in Rc)
pc = i_rlc * (v + i_rlc * rc)
fit_pc = fit_i * fit_vc

# Power delivered to ideal inductance
fit_pl = fit_pc - fit_pr

fit_vl = fit_vc - fit_i * r

ec_max = c * vmax**2 / 2
el_max = scipy.integrate.trapz(fit_pl[:fit_imax_pos], dx=1 / sample_rate)

# Energy left in the capacitor if we cut off current at Imax
cutoff_final_energy = c * fit_vc[fit_imax_pos]**2 / 2

print 'Pre-trigger samples     %d' % start
print 'Peak voltage:           %5.1f V' % vmax
print 'Peak current:           %5.1f A' % imax
print 'Peak power out of cap:  %5.1f kW' % (max(fit_pc) / 1e3)
print 'Peak power to inductor: %5.1f kW' % (max(fit_pl) / 1e3)
print 'Mean pos inductor power:%5.1f kW' % (el_max / fit_imax_time / 1e3)
print 'Peak ohmic loss:        %5.1f kW' % (max(fit_pr) / 1e3)
print 'Natural frequency:      %5.1f Hz' % w0
print 'Damped frequency:       %5.1f Hz' % wd
print 'Alpha (zeta*w0):        %7.3f' % a
print 'Damping ratio (zeta):   %7.3f' % z
print 'Current curve scale:   %8.3f' % iscale
print 'Capacitance (each):    %6.1f uF' % (c * 1e6 / 2)
print 'Inductance:             %5.1f uH' % (L * 1e6)
print 'Sense resistor:          %2.0f milliohms' % (rsense * 1e3)
print 'Capacitor resistance:    %5.2f milliohms' % (rc * 1e3)
print 'Inductor resistance:    %6.2f milliohms' % (rl * 1e3)
print 'Total Resistance:       %6.2f milliohms' % (r * 1e3)
print 'Initial energy:         %6.2f J' % (0.5 * c * vmax**2)
print 'Total energy delivered: %6.2f J' % scipy.integrate.trapz(
    pc,
    dx=1 / sample_rate)
print 'Total loss:             %6.2f J' % scipy.integrate.trapz(
    fit_pr,
    dx=1 / sample_rate)
print 'Final inductor energy:  %6.2f J' % scipy.integrate.trapz(
    fit_pl,
    dx=1 / sample_rate)
print 'Max inductor energy:    %6.2f J' % el_max

# Efficiency if we use all energy in the inductor
print 'Max efficiency:          %4.1f%%' % (el_max / ec_max * 100)

# Efficiency if we cut off current at Imax, so the capacitor does not fully discharge
print 'Cutoff efficiency:       %4.1f%%' % (el_max / (
    ec_max - cutoff_final_energy) * 100)

fig = figure()
fig.canvas.set_window_title(sys.argv[1])
subplot(311)
plot(tt, i_rlc, 'g', tt, fit_i, 'r')
vlines(fit_imax_time, 0, fit_imax)
grid(True)
ylabel('Current (A)')
legend(['Measured', 'Best fit'])
subplot(312)
plot(tt, v, 'g', tt, fit_v, 'r')
vlines(fit_imax_time, 0, fit_v[fit_imax_pos])
grid(True)
ylabel('Voltage (V)')
legend(['Measured', 'Best fit'])
subplot(313)
plot(tt, fit_pc / 1e3, 'g', tt, fit_pl / 1e3, 'r', tt, fit_pr / 1e3, 'b')
vlines(fit_imax_time, 0, fit_pr[fit_imax_pos] / 1e3)
grid(True)
legend(['Out of capacitor', 'Into inductor', 'Lost in resistance'])
ylabel('Power (kW)')
xlabel('Time (s)')
show()
