import numpy as np
import pandas as pd
import dearpygui.dearpygui as dpg
from pathlib import Path
from scipy.signal import savgol_filter
from sklearn.cross_decomposition import PLSRegression
from sklearn.model_selection import KFold, GridSearchCV, cross_val_predict
from sklearn.metrics import mean_squared_error, r2_score
from sklearn.base import BaseEstimator, RegressorMixin

# ============================================================
# DATASET LOADING (Same logic as before)
# ============================================================
def build_dataset(csv_path, data_dir):
    try:
        df = pd.read_csv(csv_path)
        df.columns = [c.lower().strip() for c in df.columns]
        id_col = next((c for c in df.columns if any(x in c for x in ["sample", "solution", "num"])), None)
        conc_col = next((c for c in df.columns if any(x in c for x in ["con", "ppm", "actual"])), None)
        if not id_col or not conc_col: return None, None
        x_raw, y_out = [], []
        for _, row in df.iterrows():
            try:
                s_num = int(''.join(filter(str.isdigit, str(row[id_col]))))
                fpath = data_dir / f"{s_num}.csv"
                if fpath.exists():
                    spec = np.mean(pd.read_csv(fpath).iloc[:, 1:].values.astype(float), axis=0)
                    it = float(row.get('integration time(ms)', 1000))
                    x_raw.append(spec / (it + 1e-8))
                    y_out.append(float(row[conc_col]))
            except: continue
        return np.array(x_raw, dtype=object), np.array(y_out)
    except: return None, None

# ============================================================
# PROCESSING & MODELING
# ============================================================
def apply_pipeline(spectrum, cfg):
    """Returns a dictionary of all steps for the combined chart."""
    steps = {"Raw": spectrum.copy()}
    
    # 1. Align/Crop
    n = len(spectrum)
    mid = n // 2
    if cfg['do_align']:
        peak_idx = np.argmax(spectrum)
        shift = mid - peak_idx
        current = np.roll(spectrum, shift)
    else: current = spectrum.copy()
    current = current[max(0, mid-cfg['hw']) : min(n, mid+cfg['hw'])]
    steps["Aligned"] = current.copy()

    # 2. Baseline
    if cfg['do_base']:
        from scipy.sparse import diags, linalg
        y, L = current, len(current)
        D = diags([1, -2, 1], [0, 1, 2], shape=(L-2, L))
        w, lam, p = np.ones(L), 1e6, 0.005
        for _ in range(5):
            z = linalg.spsolve(diags(w) + lam * D.T @ D, w * y)
            w = p * (y > z) + (1 - p) * (y <= z)
        current = np.clip(y - z, 0, None)
    steps["Baseline"] = current.copy()

    # 3. Smooth
    if cfg['do_smooth']:
        current = savgol_filter(current, cfg['s_w'], cfg['s_p'])
    steps["Smooth"] = current.copy()

    # 4. Derivative
    if cfg['do_deriv']:
        current = savgol_filter(current, cfg['d_w'], cfg['d_p'], deriv=1)
    steps["Derivative"] = current.copy()

    # 5. SNV
    if cfg['do_snv']:
        current = (current - np.mean(current)) / (np.std(current) + 1e-8)
    steps["SNV"] = current.copy()

    return current, steps

class PLSRWrapper(BaseEstimator, RegressorMixin):
    def __init__(self, n_components=1): self.n_components = n_components
    def fit(self, X, y):
        self.pls_ = PLSRegression(n_components=self.n_components, scale=True)
        self.pls_.fit(X, np.log1p(np.clip(y, 0, None))); return self
    def predict(self, X): return np.expm1(self.pls_.predict(X).ravel())

# ============================================================
# UI CLASS
# ============================================================
class SpectroscopyLabV7:
    def __init__(self, datasets):
        self.datasets = datasets
        self.mode = list(datasets.keys())[0]
        self.idx = 0
        self.cfg = {'do_align':True, 'hw':2000, 'do_base':False, 'do_smooth':True, 's_w':31, 's_p':2, 'do_deriv':True, 'd_w':31, 'd_p':3, 'do_snv':True}
        self.plot_visibility = {k: True for k in ["Raw", "Aligned", "Baseline", "Smooth", "Derivative", "SNV"]}
        self.normalize_view = False

    def update_plot(self):
        X_raw, _ = self.datasets[self.mode]
        _, steps = apply_pipeline(X_raw[self.idx], self.cfg)
        
        for name, data in steps.items():
            tag = f"series_{name}"
            if dpg.does_item_exist(tag):
                if self.plot_visibility[name]:
                    display_data = data
                    if self.normalize_view:
                        d_min, d_max = display_data.min(), display_data.max()
                        display_data = (display_data - d_min) / (d_max - d_min + 1e-8)
                    dpg.set_value(tag, [list(range(len(display_data))), list(display_data)])
                    dpg.configure_item(tag, show=True)
                else:
                    dpg.configure_item(tag, show=False)
        dpg.fit_axis_data("x_axis")
        dpg.fit_axis_data("y_axis")

    def run_training(self):
        X_raw, y = self.datasets[self.mode]
        X_proc = np.array([apply_pipeline(s, self.cfg)[0] for s in X_raw])
        kf = KFold(n_splits=5, shuffle=True, random_state=42)
        gs = GridSearchCV(PLSRWrapper(), {"n_components": range(1, 6)}, cv=kf).fit(X_proc, y)
        y_pred = cross_val_predict(gs.best_estimator_, X_proc, y, cv=kf)
        metrics = f"Best Comp: {gs.best_params_['n_components']}\nR²: {r2_score(y, y_pred):.3f}\nRMSE: {np.sqrt(mean_squared_error(y, y_pred)):.2f} ppm"
        dpg.set_value("txt_stats", metrics)
        self.update_plot()

    def _sync(self):
        for k in ['align','base','smooth','deriv','snv']: self.cfg[f'do_{k}'] = dpg.get_value(f"cfg_{k}")
        self.cfg.update({'s_w':dpg.get_value("cfg_sw"), 's_p':dpg.get_value("cfg_sp"), 'd_w':dpg.get_value("cfg_dw"), 'd_p':dpg.get_value("cfg_dp")})
        for k in self.plot_visibility: self.plot_visibility[k] = dpg.get_value(f"vis_{k}")
        self.normalize_view = dpg.get_value("vis_norm")
        self.run_training()

    def show(self):
        dpg.create_context()
        with dpg.window(label="Unified Spectroscopy Lab v7.0", width=1600, height=900):
            with dpg.group(horizontal=True):
                # LEFT SIDEBAR (Scrollable)
                with dpg.child_window(width=320, border=True):
                    dpg.add_text("ALGORITHM CONTROL", color=[100, 200, 255])
                    dpg.add_checkbox(label="Alignment & Crop", tag="cfg_align", default_value=True)
                    dpg.add_checkbox(label="ALS Baseline", tag="cfg_base", default_value=False)
                    dpg.add_checkbox(label="Smoothing", tag="cfg_smooth", default_value=True)
                    with dpg.group(horizontal=True):
                        dpg.add_input_int(tag="cfg_sw", default_value=31, width=80); dpg.add_text("Win")
                        dpg.add_input_int(tag="cfg_sp", default_value=2, width=80); dpg.add_text("Poly")
                    dpg.add_checkbox(label="Derivative", tag="cfg_deriv", default_value=True)
                    with dpg.group(horizontal=True):
                        dpg.add_input_int(tag="cfg_dw", default_value=31, width=80); dpg.add_text("Win")
                        dpg.add_input_int(tag="cfg_dp", default_value=3, width=80); dpg.add_text("Poly")
                    dpg.add_checkbox(label="SNV", tag="cfg_snv", default_value=True)
                    
                    dpg.add_spacer(height=10)
                    dpg.add_button(label="APPLY & RETRAIN", callback=self._sync, width=-1, height=40)
                    dpg.add_text("", tag="txt_stats", color=[150, 255, 150])
                    
                    dpg.add_separator()
                    dpg.add_text("PLOT VISIBILITY", color=[255, 200, 100])
                    dpg.add_checkbox(label="Normalize All (0-1 Scale)", tag="vis_norm", callback=self.update_plot)
                    for k in self.plot_visibility:
                        dpg.add_checkbox(label=f"Show {k}", tag=f"vis_{k}", default_value=True, callback=self.update_plot)

                    dpg.add_separator()
                    dpg.add_text("SAMPLES", color=[100, 200, 255])
                    _, y = self.datasets[self.mode]
                    items = [f"ID {i}: {v:.1f} ppm" for i, v in enumerate(y)]
                    dpg.add_listbox(items, width=-1, num_items=12, callback=lambda s, a: [setattr(self,'idx',items.index(a)), self.update_plot()])

                # MAIN CHART AREA
                with dpg.child_window(width=-1, border=False):
                    with dpg.plot(label="Combined Pipeline Evolution", width=-1, height=-1):
                        dpg.add_plot_legend()
                        dpg.add_plot_axis(dpg.mvXAxis, label="Pixel Index / Time", tag="x_axis")
                        dpg.add_plot_axis(dpg.mvYAxis, label="Magnitude", tag="y_axis")
                        # Pre-add series with unique colors
                        colors = [[255,255,255],[255,0,0],[0,255,0],[0,255,255],[255,0,255],[255,255,0]]
                        for i, name in enumerate(self.plot_visibility.keys()):
                            dpg.add_line_series([], [], label=name, parent="y_axis", tag=f"series_{name}")

        dpg.create_viewport(title='Unified Spectra Explorer', width=1650, height=1000)
        dpg.setup_dearpygui(); dpg.show_viewport()
        self.run_training()
        dpg.start_dearpygui(); dpg.destroy_context()

if __name__ == "__main__":
    datasets = {}
    ra, ya = build_dataset(Path("data/real_data/chla_data.csv"), Path("data/chl_a"))
    if ra is not None: datasets["Chl-A"] = (ra, ya)
    if datasets: SpectroscopyLabV7(datasets).show()
    else: print("Check your data paths.")