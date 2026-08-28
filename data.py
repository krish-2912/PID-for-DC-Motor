import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit

# ── Load raw log ──────────────────────────────────────────────────────────────
with open(r"/home/krish/pid data for dc motor/BR_MOTOR/BR_MOTOR.txt") as f:
    raw = f.read()

# ── Parse 2-column CSV blocks (Test 1 & Test 3) ───────────────────────────────
def parse_csv_block(text, header, ncols=2):
    lines, capture, data = text.split('\n'), False, []
    for line in lines:
        if header in line:
            capture = True
            continue
        if not capture:
            continue
        line = line.strip()
        if line.startswith('#') or line == '':
            continue
        if line.startswith('---') or line.startswith('==='):
            capture = False
            continue
        if ',' in line:
            try:
                row = [float(x) for x in line.split(',')]
                if len(row) == ncols:
                    data.append(row)
            except ValueError:
                continue
    return np.array(data) if data else np.array([]).reshape(0, ncols)

# ── Parse step blocks (Test 2) — 3 columns: time_ms, pwm, rpm ─────────────────
def parse_step_blocks(text):
    blocks  = []
    current = []
    capture = False

    for line in text.split('\n'):
        line = line.strip()

        if 'time_ms,pwm,rpm' in line:
            if current:
                blocks.append(np.array(current))
            current = []
            capture = True
            continue

        if not capture:
            continue
        if line.startswith('#') or line == '':
            continue
        if line.startswith('---') or line.startswith('==='):
            capture = False
            continue
        if ',' in line:
            try:
                row = [float(x) for x in line.split(',')]
                if len(row) == 3:
                    current.append(row)
            except ValueError:
                continue

    if current:
        blocks.append(np.array(current))

    return blocks

# ── Load all test data ────────────────────────────────────────────────────────
dz    = parse_csv_block(raw, "pwm,rpm",     ncols=2)
ramp  = parse_csv_block(raw, "pwm,rpm_avg", ncols=2)
steps = parse_step_blocks(raw)

print(f"Dead zone points : {len(dz)}")
print(f"Step blocks found: {len(steps)}")
print(f"Ramp points      : {len(ramp)}")

# ── TEST 1 — Dead Zone Plot ───────────────────────────────────────────────────
if len(dz) > 0:
    plt.figure(figsize=(10, 4))
    plt.plot(dz[:, 0], dz[:, 1], 'o-', ms=4)
    plt.axhline(5, color='r', ls='--', label='Threshold 5 RPM')
    plt.xlabel("PWM")
    plt.ylabel("RPM")
    plt.title("Test 1 — Dead Zone Scan")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("/home/krish/pid data for dc motor/deadzone.png")
    plt.show()
    print("✓ Dead zone plot saved.")
else:
    print("⚠ No dead zone data found — skipping Test 1 plot.")

# ── TEST 2 — Step Response + Curve Fit ───────────────────────────────────────
def first_order(t, K, tau):
    return K * (1 - np.exp(-t / tau))

results = []

if len(steps) > 0:
    ncols_plot = 3
    nrows_plot = int(np.ceil(len(steps) / ncols_plot))
    fig, axes  = plt.subplots(nrows_plot, ncols_plot,
                               figsize=(15, 5 * nrows_plot))
    axes = np.array(axes).flatten()   # safe flatten for any grid shape

    for i, data in enumerate(steps):
        ax  = axes[i]
        t   = data[:, 0] / 1000.0    # ms → s
        rpm = data[:, 2]
        pwm = int(data[0, 1])

        rpm_final = np.mean(rpm[-10:]) if len(rpm) >= 10 else np.mean(rpm)
        t_63      = None

        for j in range(len(rpm)):
            if rpm[j] >= 0.632 * rpm_final:
                t_63 = t[j]
                break

        # Curve fit
        try:
            p0 = [rpm_final / max(pwm, 1), t_63 if t_63 else 0.3]
            popt, _ = curve_fit(
                lambda t, K, tau: pwm * first_order(t, K, tau),
                t, rpm, p0=p0, maxfev=8000
            )
            K_fit, tau_fit = popt
            fit_rpm = pwm * first_order(t, K_fit, tau_fit)
            ax.plot(t, fit_rpm, 'r--', lw=2,
                    label=f'Fit  K={K_fit:.3f}  τ={tau_fit:.3f}s')
            results.append({'pwm': pwm, 'K': K_fit, 'tau': tau_fit,
                             'rpm_ss': rpm_final, 't63': t_63})
        except Exception as e:
            print(f"⚠ Curve fit failed for PWM={pwm}: {e}")
            results.append({'pwm': pwm, 'K': None, 'tau': None,
                             'rpm_ss': rpm_final, 't63': t_63})

        ax.plot(t, rpm, 'b', lw=1.5, label='Measured')
        if t_63:
            ax.axvline(t_63, color='g', ls=':', label=f'τ ≈ {t_63:.2f}s')
        ax.set_title(f"PWM = {pwm}")
        ax.set_xlabel("Time (s)")
        ax.set_ylabel("RPM")
        ax.legend(fontsize=7)
        ax.grid(True)

    # Hide unused subplots
    for j in range(len(steps), len(axes)):
        axes[j].set_visible(False)

    plt.suptitle("Test 2 — Step Response (Red = 1st Order Fit)", fontsize=13)
    plt.tight_layout()
    plt.savefig("/home/krish/pid data for dc motor/step_response.png")
    plt.show()
    print("✓ Step response plot saved.")
else:
    print("⚠ No step response data found — skipping Test 2 plot.")

# ── TEST 3 — Linearity Map ────────────────────────────────────────────────────
if len(ramp) > 0:
    plt.figure(figsize=(8, 4))
    plt.plot(ramp[:, 0], ramp[:, 1], 's-', ms=6, label='Measured')
    m, b = np.polyfit(ramp[:, 0], ramp[:, 1], 1)
    plt.plot(ramp[:, 0], m * ramp[:, 0] + b, 'r--',
             label=f'Linear fit: RPM = {m:.2f}×PWM + {b:.1f}')
    plt.xlabel("PWM")
    plt.ylabel("Steady-State RPM")
    plt.title("Test 3 — Linearity: RPM vs PWM")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.savefig("/home/krish/pid data for dc motor/linearity.png")
    plt.show()
    print("✓ Linearity plot saved.")
else:
    print("⚠ No ramp data found — skipping Test 3 plot.")

# ── Summary ───────────────────────────────────────────────────────────────────
if results:
    print("\n===== IDENTIFIED PARAMETERS =====")
    print(f"{'PWM':>6} {'K (RPM/PWM)':>12} {'τ (s)':>8} {'RPM_ss':>8} {'t63 (s)':>8}")
    print("-" * 54)
    for r in results:
        K_str   = f"{r['K']:.4f}"   if r['K']   is not None else "   N/A"
        tau_str = f"{r['tau']:.4f}" if r['tau'] is not None else "   N/A"
        t63_str = f"{r['t63']:.4f}" if r['t63'] is not None else "   N/A"
        print(f"{r['pwm']:>6} {K_str:>12} {tau_str:>8} {r['rpm_ss']:>8.1f} {t63_str:>8}")

    Ks   = [r['K']   for r in results if r['K']   is not None]
    taus = [r['tau'] for r in results if r['tau'] is not None]

    if Ks and taus:
        print(f"\nMean K   = {np.mean(Ks):.4f} RPM/PWM  (std = {np.std(Ks):.4f})")
        print(f"Mean τ   = {np.mean(taus):.4f} s        (std = {np.std(taus):.4f})")
        print(f"\nTransfer Function:  G(s) = {np.mean(Ks):.3f} / ({np.mean(taus):.3f}·s + 1)")
    else:
        print("\n⚠ Not enough valid fits to compute mean K / τ.")
else:
    print("⚠ No results to summarize.")