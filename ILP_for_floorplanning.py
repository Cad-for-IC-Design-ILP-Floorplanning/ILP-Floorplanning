# ---------------------------------------------------------------------
# ILP floorplanner: rotation + aspect-ratio range (function: solve_floorplan_with_rotation_range)
#
# Goal:
#   Minimize chip height `ystar` while allowing chip width `xstar` to float
#   within the user-specified aspect ratio range rmin <= xstar/ystar <= rmax.
#
# Key modeling points:
#   - Each module i has lower-left coordinates (x[i], y[i]) and a binary
#     rotation variable z[i]: 0 = original (w,h), 1 = rotated 90° (h,w).
#   - Non-overlap is enforced pairwise using binaries xrel[(a,b)], yrel[(a,b)]
#     combined with Big-M offsets to activate/deactivate the appropriate
#     relative-position inequalities.
#   - The formulation is slicing-free and uses linearized effective sizes
#     (so the ILP remains linear).
#
# Output:
#   Returns solver status plus coordinates, rotation bits and pairwise encodings.
# ---------------------------------------------------------------------

import tkinter as tk
from tkinter import ttk, messagebox

# ---- ILP solver (PuLP) ----
import pulp


def solve_floorplan_with_rotation_range(modules, rmin=0.5, rmax=2.0, msg=False, timeLimit=None):

    if not modules:
        return {'status': 'No modules', 'ystar': None, 'xstar': None, 'coords': {}, 'z': {}, 'pairs': {}, 'M': 0}

    names = [m['name'] for m in modules]
    w = {m['name']: float(m['w']) for m in modules}
    h = {m['name']: float(m['h']) for m in modules}

    # Big-M (safe upper bound)
    Wsum = sum(w.values())
    Hsum = sum(h.values())
    M = max(Wsum, Hsum)

    prob = pulp.LpProblem("floorplan_rot_ar_range", pulp.LpMinimize)

    # Continuous coordinate variables (lower-left corner)
    x = pulp.LpVariable.dicts("x", names, lowBound=0, cat='Continuous')
    y = pulp.LpVariable.dicts("y", names, lowBound=0, cat='Continuous')

    # Chip dims
    xstar = pulp.LpVariable("xstar", lowBound=0, cat='Continuous')
    ystar = pulp.LpVariable("ystar", lowBound=0, cat='Continuous')

    # Rotation variables
    z = {}
    for m in modules:
        name = m['name']
        rotation = m.get('rotation', 'free')
        if rotation == 'fixed_original':
            z[name] = pulp.LpVariable(f"z_{name}", lowBound=0, upBound=0, cat='Continuous')
        elif rotation == 'fixed_rotated':
            z[name] = pulp.LpVariable(f"z_{name}", lowBound=1, upBound=1, cat='Continuous')
        else:
            z[name] = pulp.LpVariable(f"z_{name}", cat='Binary')

    # Pairwise binaries
    xrel, yrel, pairs = {}, {}, []
    for i in range(len(names)):
        for j in range(i+1, len(names)):
            a, b = names[i], names[j]
            pairs.append((a, b))
            xrel[(a, b)] = pulp.LpVariable(f"x_{a}_{b}", cat='Binary')
            yrel[(a, b)] = pulp.LpVariable(f"y_{a}_{b}", cat='Binary')

    # Objective: minimize chip height
    prob += ystar, "minimize_chip_height"

    # Aspect ratio bounds: rmin * ystar <= xstar <= rmax * ystar
    # Defensive: ensure rmin>0 and rmin<=rmax; caller must validate
    prob += xstar >= rmin * ystar
    prob += xstar <= rmax * ystar

    # Non-overlap constraints with rotation (use effective dimensions)
    for (a, b) in pairs:
        # effective widths/heights linearized as (1 - z)*w + z*h etc.
        wa_eff_a = (1 - z[a]) * w[a] + z[a] * h[a]
        wa_eff_b = (1 - z[b]) * w[b] + z[b] * h[b]
        ha_eff_a = (1 - z[a]) * h[a] + z[a] * w[a]
        ha_eff_b = (1 - z[b]) * h[b] + z[b] * w[b]

        prob += x[a] + wa_eff_a <= x[b] + M * (xrel[(a, b)] + yrel[(a, b)])
        prob += x[b] + wa_eff_b <= x[a] + M * (1 - xrel[(a, b)] + yrel[(a, b)])
        prob += y[a] + ha_eff_a <= y[b] + M * (1 + xrel[(a, b)] - yrel[(a, b)])
        prob += y[b] + ha_eff_b <= y[a] + M * (2 - xrel[(a, b)] - yrel[(a, b)])

    # Chip bounds
    for a in names:
        eff_w = (1 - z[a]) * w[a] + z[a] * h[a]
        eff_h = (1 - z[a]) * h[a] + z[a] * w[a]
        prob += x[a] + eff_w <= xstar
        prob += y[a] + eff_h <= ystar

    # Symmetry break (optional): pin first module at origin
    prob += x[names[0]] == 0
    prob += y[names[0]] == 0

    # Solver options
    solver_kwargs = {'msg': bool(msg)}
    if timeLimit is not None:
        solver_kwargs['timeLimit'] = int(timeLimit)
    solver = pulp.PULP_CBC_CMD(**solver_kwargs)

    res = prob.solve(solver)
    status = pulp.LpStatus[res]
    if status != 'Optimal':
        # attempt to return partial values if available
        coords = {a: (pulp.value(x[a]), pulp.value(y[a])) for a in names}
        zs = {a: (None if pulp.value(z[a]) is None else int(round(pulp.value(z[a])))) for a in names}
        return {'status': status, 'ystar': None, 'xstar': None, 'coords': coords, 'z': zs, 'pairs': {}, 'M': M}

    coords = {a: (pulp.value(x[a]), pulp.value(y[a])) for a in names}
    zsol = {a: int(round(pulp.value(z[a]))) for a in names}
    pairsol = {p: (int(round(pulp.value(xrel[p]))), int(round(pulp.value(yrel[p])))) for p in pairs}

    return {'status': status, 'ystar': float(pulp.value(ystar)), 'xstar': float(pulp.value(xstar)),
            'coords': coords, 'z': zsol, 'pairs': pairsol, 'M': M}


# ---------------- Tkinter GUI ----------------
class FloorplanApp(tk.Tk):
    def __init__(self):
        super().__init__()
        self.title("Dynamic ILP Floorplanner (rotation allowed, AR range)")
        self.geometry("1200x820")
        self.minsize(900, 600)

        # Initialize with default modules
        self.modules = [
            {'name': 'm1', 'w': 4.0, 'h': 5.0, 'rotation': 'free'},
            {'name': 'm2', 'w': 3.0, 'h': 7.0, 'rotation': 'free'},
            {'name': 'm3', 'w': 6.0, 'h': 4.0, 'rotation': 'free'},
            {'name': 'm4', 'w': 7.0, 'h': 7.0, 'rotation': 'free'},
        ]
        self.module_counter = 5
        self._build_ui()

    def _build_ui(self):
        # Main container
        main_frame = ttk.PanedWindow(self, orient='horizontal')
        main_frame.pack(fill='both', expand=True, padx=5, pady=5)

        # Left panel for controls
        left_frame = ttk.Frame(main_frame, padding=10)
        main_frame.add(left_frame, weight=1)

        # Module management section
        module_frame = ttk.LabelFrame(left_frame, text="Module Management", padding=10)
        module_frame.pack(fill='both', expand=True)

        # Aspect ratio inputs
        ar_row = ttk.Frame(left_frame)
        ar_row.pack(fill='x', pady=6)
        ttk.Label(ar_row, text="Aspect ratio range (width / height):", width=28).pack(side='left')
        self.rmin_var = tk.StringVar(value="0.8")
        self.rmax_var = tk.StringVar(value="1.25")
        ttk.Entry(ar_row, width=8, textvariable=self.rmin_var).pack(side='left', padx=4)
        ttk.Label(ar_row, text="to").pack(side='left')
        ttk.Entry(ar_row, width=8, textvariable=self.rmax_var).pack(side='left', padx=4)
        

        # Control buttons
        btn_frame = ttk.Frame(module_frame)
        btn_frame.pack(fill='x', pady=(0, 10))

        ttk.Button(btn_frame, text="Add Module", command=self.add_module).pack(side='left', padx=(0, 5))
        ttk.Button(btn_frame, text="Remove Selected", command=self.remove_module).pack(side='left', padx=(0, 5))
        ttk.Button(btn_frame, text="Clear All", command=self.clear_modules).pack(side='left', padx=(0, 5))

        # Module list with scrollbar
        list_frame = ttk.Frame(module_frame)
        list_frame.pack(fill='both', expand=True, pady=(0, 10))

        # Headers
        header_frame = ttk.Frame(list_frame)
        header_frame.pack(fill='x', pady=(0, 5))
        ttk.Label(header_frame, text="Sel", width=4, font=('Helvetica', 9, 'bold')).pack(side='left')
        ttk.Label(header_frame, text="Name", width=8, font=('Helvetica', 9, 'bold')).pack(side='left')
        ttk.Label(header_frame, text="Width", width=6, font=('Helvetica', 9, 'bold')).pack(side='left')
        ttk.Label(header_frame, text="Height", width=6, font=('Helvetica', 9, 'bold')).pack(side='left')
        ttk.Label(header_frame, text="Rotation", width=12, font=('Helvetica', 9, 'bold')).pack(side='left')

        # Scrollable frame for modules
        self.canvas_frame = tk.Canvas(list_frame, height=200)
        self.scrollbar = ttk.Scrollbar(list_frame, orient="vertical", command=self.canvas_frame.yview)
        self.scrollable_frame = ttk.Frame(self.canvas_frame)

        self.scrollable_frame.bind(
            "<Configure>",
            lambda e: self.canvas_frame.configure(scrollregion=self.canvas_frame.bbox("all"))
        )

        self.canvas_frame.create_window((0, 0), window=self.scrollable_frame, anchor="nw")
        self.canvas_frame.configure(yscrollcommand=self.scrollbar.set)

        self.canvas_frame.pack(side="left", fill="both", expand=True)
        self.scrollbar.pack(side="right", fill="y")

        # Control buttons section
        control_frame = ttk.LabelFrame(left_frame, text="Controls", padding=10)
        control_frame.pack(fill='x', pady=(10, 0))

        ttk.Button(control_frame, text="Solve & Draw", command=self.on_solve).pack(fill='x', pady=3)
        ttk.Button(control_frame, text="Load Default Example", command=self.load_defaults).pack(fill='x', pady=3)

        # Status section
        status_frame = ttk.LabelFrame(left_frame, text="Results", padding=10)
        status_frame.pack(fill='x', pady=(10, 0))

        ttk.Label(status_frame, text="Solver Status:").pack(anchor='w')
        self.status_var = tk.StringVar(value="Ready")
        ttk.Label(status_frame, textvariable=self.status_var, foreground='blue', 
                 font=('Helvetica', 9, 'bold')).pack(anchor='w')

        ttk.Label(status_frame, text="Chip Width (x*):").pack(anchor='w', pady=(8,0))
        self.xstar_var = tk.StringVar(value="N/A")
        ttk.Label(status_frame, textvariable=self.xstar_var, 
                 font=('Helvetica', 12, 'bold'), foreground='green').pack(anchor='w')

        ttk.Label(status_frame, text="Chip Height (y*):").pack(anchor='w', pady=(8,0))
        self.ystar_var = tk.StringVar(value="N/A")
        ttk.Label(status_frame, textvariable=self.ystar_var, 
                 font=('Helvetica', 12, 'bold'), foreground='green').pack(anchor='w')

        ttk.Label(status_frame, text="Module Count:").pack(anchor='w', pady=(8,0))
        self.count_var = tk.StringVar()
        ttk.Label(status_frame, textvariable=self.count_var, 
                 font=('Helvetica', 9)).pack(anchor='w')

        # Right panel for canvas
        right_frame = ttk.Frame(main_frame, padding=10)
        main_frame.add(right_frame, weight=2)

        canvas_label_frame = ttk.LabelFrame(right_frame, text="Floorplan Visualization", padding=5)
        canvas_label_frame.pack(fill='both', expand=True)

        self.canvas = tk.Canvas(canvas_label_frame, bg='white', relief='sunken', bd=2)
        self.canvas.pack(fill='both', expand=True)

        # Initialize the module list
        self.refresh_module_list()

    def add_module(self):
        """Add a new module."""
        name = f"m{self.module_counter}"
        width = 5.0
        height = 5.0
        rotation = 'free'
        existing_names = [m['name'] for m in self.modules]
        while name in existing_names:
            self.module_counter += 1
            name = f"m{self.module_counter}"
        self.modules.append({'name': name, 'w': width, 'h': height, 'rotation': rotation})
        self.module_counter += 1
        self.refresh_module_list()

    def remove_module(self):
        """Remove selected module(s)."""
        if not hasattr(self, 'module_entries') or not self.module_entries:
            return
        to_remove = []
        for i, entry_data in enumerate(self.module_entries):
            if entry_data[1].get():
                to_remove.append(i)
        if not to_remove:
            return
        for i in reversed(to_remove):
            del self.modules[i]
        self.refresh_module_list()

    def clear_modules(self):
        """Clear all modules."""
        self.modules.clear()
        self.refresh_module_list()

    def load_defaults(self):
        """Load default example modules."""
        self.modules = [
            {'name': 'm1', 'w': 4.0, 'h': 5.0, 'rotation': 'free'},
            {'name': 'm2', 'w': 3.0, 'h': 7.0, 'rotation': 'free'},
            {'name': 'm3', 'w': 6.0, 'h': 4.0, 'rotation': 'free'},
            {'name': 'm4', 'w': 7.0, 'h': 7.0, 'rotation': 'free'},
        ]
        self.refresh_module_list()

    def refresh_module_list(self):
        """Refresh the module list display."""
        for widget in self.scrollable_frame.winfo_children():
            widget.destroy()
        self.module_entries = []
        for i, module in enumerate(self.modules):
            row_frame = ttk.Frame(self.scrollable_frame)
            row_frame.pack(fill='x', pady=2)
            var = tk.BooleanVar()
            ttk.Checkbutton(row_frame, variable=var, width=2).pack(side='left')
            name_var = tk.StringVar(value=module['name'])
            ttk.Entry(row_frame, width=8, textvariable=name_var).pack(side='left', padx=2)
            w_var = tk.StringVar(value=str(module['w']))
            ttk.Entry(row_frame, width=6, textvariable=w_var).pack(side='left', padx=2)
            h_var = tk.StringVar(value=str(module['h']))
            ttk.Entry(row_frame, width=6, textvariable=h_var).pack(side='left', padx=2)
            rotation_var = tk.StringVar(value=module.get('rotation', 'free'))
            rotation_combo = ttk.Combobox(row_frame, width=10, textvariable=rotation_var, 
                                        values=['free', 'fixed_original', 'fixed_rotated'],
                                        state='readonly')
            rotation_combo.pack(side='left', padx=2)
            self.module_entries.append((module, var, name_var, w_var, h_var, rotation_var))
        self.count_var.set(f"{len(self.modules)} modules")

    def validate_and_update_modules(self):
        """Validate and update modules from the entry fields."""
        updated_modules = []
        names_used = set()
        for entry_data in self.module_entries:
            module, var, name_var, w_var, h_var, rotation_var = entry_data
            name = name_var.get().strip()
            if not name:
                messagebox.showerror("Error", "Module name cannot be empty")
                return False
            if name in names_used:
                messagebox.showerror("Error", f"Duplicate module name: '{name}'")
                return False
            names_used.add(name)
            width_str = w_var.get().strip()
            height_str = h_var.get().strip()
            if not width_str or not height_str:
                messagebox.showerror("Error", f"Width and height must be specified for module '{name}'")
                return False
            width = float(width_str)
            height = float(height_str)
            rotation = rotation_var.get()
            if width <= 0 or height <= 0:
                messagebox.showerror("Error", f"Width and height must be positive for module '{name}'")
                return False
            updated_modules.append({'name': name, 'w': width, 'h': height, 'rotation': rotation})
        self.modules = updated_modules
        return True

    def on_solve(self):
        """Solve the floorplanning problem."""
        if not self.modules:
            messagebox.showwarning("Warning", "Please add at least one module.")
            return
        if not self.validate_and_update_modules():
            return

        # Validate aspect ratio inputs
        try:
            rmin = float(self.rmin_var.get())
            rmax = float(self.rmax_var.get())
            if not (rmin > 0 and rmax > 0 and rmin <= rmax):
                raise ValueError
        except Exception:
            messagebox.showerror("Error", "Aspect ratio range must be two positive numbers with rmin <= rmax")
            return

        self.status_var.set("Solving...")
        self.ystar_var.set("Computing...")
        self.xstar_var.set("Computing...")
        self.update_idletasks()

        sol = solve_floorplan_with_rotation_range(self.modules, rmin=rmin, rmax=rmax)

        if sol['status'] != 'Optimal':
            self.status_var.set(f"Failed: {sol['status']}")
            self.ystar_var.set("N/A")
            self.xstar_var.set("N/A")
            messagebox.showwarning("Solver Status", f"Solver ended with status: {sol['status']}")
            return

        self.status_var.set("Optimal Solution Found")
        self.ystar_var.set(f"{sol['ystar']:.4f}")
        self.xstar_var.set(f"{sol['xstar']:.4f}")

        # Draw the solution
        self.draw_solution(self.modules, sol)

    def draw_solution(self, modules, sol):
        """Draw the floorplan solution on the canvas."""
        self.canvas.delete('all')
        if not sol['coords']:
            return
        coords = sol['coords']
        z = sol['z']
        ystar = sol['ystar'] or 1.0
        xstar = sol['xstar'] or ystar

        # Get canvas dimensions
        self.canvas.update_idletasks()
        cw = self.canvas.winfo_width()
        ch = self.canvas.winfo_height()
        pad = 40

        # Calculate scale to fit the chip in the canvas
        available_w = cw - 2 * pad
        available_h = ch - 2 * pad - 140  # Leave space for legend
        scale = min(available_w / xstar, available_h / ystar)

        # Draw chip boundary
        bx = pad
        by = pad
        bwidth = xstar * scale
        bheight = ystar * scale
        self.canvas.create_rectangle(bx, by, bx + bwidth, by + bheight, 
                                   outline='black', width=3, fill='#f0f0f0')

        # Add chip info
        self.canvas.create_text(bx + bwidth/2, by - 20, 
                              text=f"Chip Size: {xstar:.3f} × {ystar:.3f}", 
                              font=('Helvetica', 12, 'bold'), anchor='center')

        # Color palette for modules
        colors = ['#FFB6C1', '#87CEFA', '#98FB98', '#FFD700', '#DDA0DD', 
                 '#FFA07A', '#B0E0E6', '#F0E68C', '#DEB887', '#F5DEB3']

        legend_info = []
        for i, m in enumerate(modules):
            name = m['name']
            w0, h0 = m['w'], m['h']
            zi = z.get(name, 0)
            eff_w = (1 - zi) * w0 + zi * h0
            eff_h = (1 - zi) * h0 + zi * w0
            mx, my = coords.get(name, (0.0, 0.0))
            cx1 = bx + mx * scale
            cy1 = by + (ystar - (my + eff_h)) * scale
            cx2 = cx1 + eff_w * scale
            cy2 = cy1 + eff_h * scale
            color = colors[i % len(colors)]
            self.canvas.create_rectangle(cx1, cy1, cx2, cy2, 
                                       fill=color, outline='black', width=2)
            label_text = f"{name}\n{w0:.1f}×{h0:.1f}"
            if zi:
                label_text += f"\n(Rotated)"
            self.canvas.create_text((cx1 + cx2)/2, (cy1 + cy2)/2,
                                  text=label_text,
                                  font=('Helvetica', 9, 'bold'),
                                  justify='center')
            rotation_info = m.get('rotation', 'free')
            if rotation_info == 'fixed_original':
                rotation_status = "Fixed Original"
            elif rotation_info == 'fixed_rotated':
                rotation_status = "Fixed Rotated"
            else:
                rotation_status = "Rotated 90°" if zi else "Original"
            legend_info.append(f"{name}: ({mx:.2f}, {my:.2f}) - {rotation_status}")

        # Draw legend
        legend_y = by + bheight + 20
        self.canvas.create_text(bx, legend_y, 
                              text="Module Positions and Rotations:",
                              font=('Helvetica', 10, 'bold'),
                              anchor='nw')
        legend_text = "\n".join(legend_info)
        self.canvas.create_text(bx, legend_y + 20,
                              text=legend_text,
                              font=('Courier', 9),
                              anchor='nw')

        # Add coordinate system info
        coord_info = "Coordinate System: (0,0) at bottom-left of chip"
        self.canvas.create_text(bx + bwidth - 10, legend_y,
                              text=coord_info,
                              font=('Helvetica', 8),
                              anchor='ne')


# ---- Main ----
if __name__ == "__main__":
    app = FloorplanApp()
    app.mainloop()
