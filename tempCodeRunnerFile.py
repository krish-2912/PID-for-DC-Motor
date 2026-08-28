import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
import re

# ── Load raw log ──────────────────────────────────────────
with open("FL_motor.txt") as f:
    raw = f.read()

# ── Parse each section ────────────────────────────────────
def parse_csv_block(text, header):
    """Extract CSV rows after a header line"""
    lines, capture, data = text.split('\n'), False, []
    for line in lines:
        if header in line:           capture = True; continue
        if capture and line.startswith('#'): continue
        if capture and ',' in line:
            try: data.append([float(x) for x in line.split(',')])
            except: capture = False
    return np.array(data) if data else np.array([]).reshape(0,2)

dz    = parse_csv_block(raw, "pwm,rpm")
ramp  = parse_csv_block(raw, "pwm,rpm_avg")

# Separate step blocks
step_blocks = re.split(r'# Step \d+/\d+.*\n.*time_ms,pwm,rpm', raw)[1:]
steps = []
for blk in step_blocks:
    rows = []
    for line in blk.split('\n'):
        if line.startswith('#') or ',' not in line: continue
        try: rows.append([float(x) for x in line.split(',')])
        except: pass
    if rows: steps.append(np.array(rows))

# ── 1. Dead Zone Plot ─────────────────────────────────────
plt.figure(figsize=(10, 4))
plt.plot(dz[:, 0], dz[:, 1], 'o-', ms=4)
plt.axhline(5, color='r', ls='--', label='Threshold 5 RPM')
plt.xlabel("PWM"); plt.ylabel("RPM")
plt.title("Test 1 — Dead Zone Scan")
plt.legend(); plt.grid(True); plt.tight_layout()
plt.savefig("deadzone.png"); plt.show()

# ── 2. Step Response + Curve Fit ─────────────────────────
def first_order(t, K, tau):
    return K * (1 - np.exp(-t / tau))

fig, axes = plt.subplots(2, 3, figsize=(15, 8))
results = []

for i, (data, ax) in enumerate(zip(steps, axes.flat)):
    t   = data[:, 0] / 1000.0   # ms → s
    rpm = data[:, 2]
    pwm = int(data[0, 1])

    rpm_final = np.mean(rpm[-10:])   # last 10 samples = steady state
    t_63      = None

    # Find τ as time to reach 63.2% of final
    for j in range(len(rpm)):
        if rpm[j] >= 0.632 * rpm_final:
            t_63 = t[j]; break

    # Curve fit
    try:
        p0 = [rpm_final / pwm, t_63 or 0.3]
        popt, _ = curve_fit(lambda t, K, tau: pwm * first_order(t, K, tau),
                            t, rpm, p0=p0, maxfev=5000)
        K_fit, tau_fit = popt
        fit_rpm = pwm * first_order(t, K_fit, tau_fit)
        ax.plot(t, fit_rpm, 'r--', lw=2, label=f'Fit K={K_fit:.3f} τ={tau_fit:.3f}s')
        results.append({'pwm': pwm, 'K': K_fit, 'tau': tau_fit,
                        'rpm_ss': rpm_final, 't63': t_63})
    except Exception as e:
        results.append({'pwm': pwm, 'K': None, 'tau': None,
                        'rpm_ss': rpm_final, 't63': t_63})

    ax.plot(t, rpm, 'b', lw=1.5, label='Measured')
    if t_63: ax.axvline(t_63, color='g', ls=':', label=f'τ ≈ {t_63:.2f}s')
    ax.set_title(f"PWM = {pwm}")
    ax.set_xlabel("Time (s)"); ax.set_ylabel("RPM")
    ax.legend(fontsize=7); ax.grid(True)

plt.suptitle("Test 2 — Step Response (Red = 1st Order Fit)", fontsize=13)
plt.tight_layout(); plt.savefig("step_response.png"); plt.show()

# ── 3. Linearity Map ──────────────────────────────────────
if len(ramp) > 0:
    plt.figure(figsize=(8, 4))
    plt.plot(ramp[:, 0], ramp[:, 1], 's-', ms=6, label='Measured')
    # Linear fit
    m, b = np.polyfit(ramp[:, 0], ramp[:, 1], 1)
    plt.plot(ramp[:, 0], m * ramp[:, 0] + b, 'r--',
             label=f'Linear fit: RPM = {m:.2f}×PWM + {b:.1f}')
    plt.xlabel("PWM"); plt.ylabel("Steady-State RPM")
    plt.title("Test 3 — Linearity: RPM vs PWM")
    plt.legend(); plt.grid(True); plt.tight_layout()
    plt.savefig("linearity.png"); plt.show()

# ── Summary ───────────────────────────────────────────────
print("\n===== IDENTIFIED PARAMETERS =====")
print(f"{'PWM':>6} {'K (RPM/PWM)':>12} {'τ (s)':>8} {'RPM_ss':>8} {'t63 (s)':>8}")
print("-" * 48)
for r in results:
    K_str   = f"{r['K']:.4f}"   if r['K']   else "  N/A "
    tau_str = f"{r['tau']:.4f}" if r['tau'] else "  N/A "
    t63_str = f"{r['t63']:.4f}" if r['t63'] else "  N/A "
    print(f"{r['pwm']:>6} {K_str:>12} {tau_str:>8} {r['rpm_ss']:>8.1f} {t63_str:>8}")

Ks   = [r['K']   for r in results if r['K']]
taus = [r['tau'] for r in results if r['tau']]
print(f"\nMean K   = {np.mean(Ks):.4f} RPM/PWM  (std={np.std(Ks):.4f})")
print(f"Mean τ   = {np.mean(taus):.4f} s        (std={np.std(taus):.4f})")
print(f"\nTransfer Function:  G(s) = {np.mean(Ks):.3f} / ({np.mean(taus):.3f}s + 1)")