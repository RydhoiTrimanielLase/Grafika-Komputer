import heapq
import tkinter as tk
import random
import math

# ==========================================
# MODUL 1: STRUKTUR DATA DISJOINT-SET
# ==========================================
class UnionFind:
    def __init__(self, n):
        self.parent = list(range(n))
        self.rank = [0] * n

    def find(self, i):
        if self.parent[i] == i:
            return i
        self.parent[i] = self.find(self.parent[i])
        return self.parent[i]

    def union(self, i, j):
        root_i = self.find(i)
        root_j = self.find(j)
        if root_i != root_j:
            if self.rank[root_i] < self.rank[root_j]:
                self.parent[root_i] = root_j
            elif self.rank[root_i] > self.rank[root_j]:
                self.parent[root_j] = root_i
            else:
                self.parent[root_j] = root_i
                self.rank[root_i] += 1
            return True
        return False


# ==========================================
# MODUL 2: MATEMATIKA KURVA BEZIER
# ==========================================
class BezierMath:
    @staticmethod
    def calculate_point(p0, p1, p2, p3, t):
        u = 1.0 - t
        tt = t * t
        uu = u * u
        uuu = uu * u
        ttt = tt * t

        x = (uuu * p0['x']) + (3 * uu * t * p1['x']) + (3 * u * tt * p2['x']) + (ttt * p3['x'])
        y = (uuu * p0['y']) + (3 * uu * t * p1['y']) + (3 * u * tt * p2['y']) + (ttt * p3['y'])
        return x, y

    @staticmethod
    def calculate_derivative(p0, p1, p2, p3, t):
        """Turunan pertama B'(t) untuk mendapatkan vektor arah (dx, dy)."""
        u = 1.0 - t
        
        # Formula Turunan Kubik: 3(1-t)^2(P1-P0) + 6(1-t)t(P2-P1) + 3t^2(P3-P2)
        term1 = 3 * u * u
        term2 = 6 * u * t
        term3 = 3 * t * t

        dx = (term1 * (p1['x'] - p0['x'])) + (term2 * (p2['x'] - p1['x'])) + (term3 * (p3['x'] - p2['x']))
        dy = (term1 * (p1['y'] - p0['y'])) + (term2 * (p2['y'] - p1['y'])) + (term3 * (p3['y'] - p2['y']))
        return dx, dy

    @staticmethod
    def estimate_length(p0, p1, p2, p3, segments=10):
        length = 0.0
        prev_x, prev_y = p0['x'], p0['y']
        for i in range(1, segments + 1):
            t = i / segments
            curr_x, curr_y = BezierMath.calculate_point(p0, p1, p2, p3, t)
            length += math.hypot(curr_x - prev_x, curr_y - prev_y)
            prev_x, prev_y = curr_x, curr_y
        return length


# ==========================================
# MODUL 3: MAP GENERATOR & PATHFINDING
# ==========================================
class MapGenerator:
    def __init__(self, num_nodes, world_bounds):
        self.num_nodes = num_nodes
        self.bounds = world_bounds
        self.nodes = []
        self.edges_data = [] 
        self.adj_list = {} 
        
        self.generate_map()
        self.build_adjacency_list()

    def generate_map(self):
        for _ in range(self.num_nodes):
            self.nodes.append({
                'x': random.randint(self.bounds[0], self.bounds[1]), 
                'y': random.randint(self.bounds[0], self.bounds[1])
            })

        candidate_edges = []
        for i in range(self.num_nodes):
            distances = []
            for j in range(self.num_nodes):
                if i != j:
                    dist = math.hypot(self.nodes[i]['x'] - self.nodes[j]['x'], 
                                      self.nodes[i]['y'] - self.nodes[j]['y'])
                    distances.append((dist, i, j))
            distances.sort(key=lambda x: x[0])
            for k in range(min(6, len(distances))):
                candidate_edges.append(distances[k])
        candidate_edges.sort(key=lambda x: x[0])

        uf = UnionFind(self.num_nodes)
        raw_edges = []
        rejected_edges = []
        for weight, u, v in candidate_edges:
            if uf.union(u, v): raw_edges.append((u, v))
            else: rejected_edges.append((u, v))

        rejected_edges = list(set([tuple(sorted(edge)) for edge in rejected_edges]))
        num_extra_edges = int(self.num_nodes * 0.15)
        raw_edges.extend(random.sample(rejected_edges, min(num_extra_edges, len(rejected_edges))))

        self.generate_bezier_controls(raw_edges)

    def generate_bezier_controls(self, raw_edges):
        for u, v in raw_edges:
            p0 = self.nodes[u]
            p3 = self.nodes[v]
            
            dx, dy = p3['x'] - p0['x'], p3['y'] - p0['y']
            dist = math.hypot(dx, dy)
            if dist == 0: continue
            
            nx, ny = -dy / dist, dx / dist
            
            if random.random() < 0.10: offset1, offset2 = 0, 0
            else:
                offset1 = random.uniform(-0.25, 0.25) * dist
                offset2 = random.uniform(-0.25, 0.25) * dist
            
            p1 = {'x': p0['x'] + (dx * 0.33) + (nx * offset1), 'y': p0['y'] + (dy * 0.33) + (ny * offset1)}
            p2 = {'x': p0['x'] + (dx * 0.67) + (nx * offset2), 'y': p0['y'] + (dy * 0.67) + (ny * offset2)}
                  
            self.edges_data.append({'u': u, 'v': v, 'p0': p0, 'p1': p1, 'p2': p2, 'p3': p3})

    def build_adjacency_list(self):
        self.adj_list = {i: [] for i in range(self.num_nodes)}
        for edge in self.edges_data:
            u, v = edge['u'], edge['v']
            weight = BezierMath.estimate_length(edge['p0'], edge['p1'], edge['p2'], edge['p3'])
            self.adj_list[u].append({'node': v, 'weight': weight})
            self.adj_list[v].append({'node': u, 'weight': weight})

    def find_shortest_path(self, start_node, target_node):
        distances = {node: float('inf') for node in range(self.num_nodes)}
        distances[start_node] = 0
        pq = [(0, start_node)]
        previous_nodes = {node: None for node in range(self.num_nodes)}

        while pq:
            current_distance, current_node = heapq.heappop(pq)
            if current_distance > distances[current_node]: continue
            if current_node == target_node: break

            for neighbor in self.adj_list[current_node]:
                next_node, weight = neighbor['node'], neighbor['weight']
                new_distance = current_distance + weight
                if new_distance < distances[next_node]:
                    distances[next_node] = new_distance
                    previous_nodes[next_node] = current_node
                    heapq.heappush(pq, (new_distance, next_node))

        path = []
        curr = target_node
        while curr is not None:
            path.append(curr)
            curr = previous_nodes[curr]
        path.reverse()
        return path if (path and path[0] == start_node) else []


# ==========================================
# MODUL 4: KAMERA
# ==========================================
class Camera:
    def __init__(self):
        self.scale = 1.0
        self.offset_x = 0.0
        self.offset_y = 0.0

    def world_to_screen(self, wx, wy):
        return (wx * self.scale) + self.offset_x, (wy * self.scale) + self.offset_y

    def screen_to_world(self, sx, sy):
        return (sx - self.offset_x) / self.scale, (sy - self.offset_y) / self.scale


# ==========================================
# MODUL 5: VIEWPORT, ANIMASI, & UI
# ==========================================
class VectorMapApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Engine Peta Spasial Vektor - Simulasi Dinamis")
        self.root.geometry("1000x700")
        self.canvas = tk.Canvas(self.root, bg="#1a1a24", highlightthickness=0)
        self.canvas.pack(fill=tk.BOTH, expand=True)
        
        self.camera = Camera()
        self.map_generator = MapGenerator(num_nodes=100, world_bounds=(-3000, 3000))
        
        # State Kamera
        self.last_mouse_x = 0
        self.last_mouse_y = 0
        
        # State Pathfinding
        self.start_node = None
        self.target_node = None
        self.path_edges = set()
        
        # State Animasi
        self.is_animating = False
        self.path_segments = []       # Urutan kurva yang harus dilalui
        self.current_segment_idx = 0  # Index kurva saat ini
        self.current_t = 0.0          # Posisi parametrik di kurva saat ini
        self.vehicle_speed = 30.0     # Kecepatan konstan di World Space
        
        self.bind_events()
        self.root.update()
        self.center_camera()
        self.camera.scale = 0.15 
        
        self.randomize_route()

    def bind_events(self):
        self.canvas.bind("<ButtonPress-1>", self.on_pan_start)
        self.canvas.bind("<B1-Motion>", self.on_pan_drag)
        self.canvas.bind("<MouseWheel>", self.on_zoom)
        self.canvas.bind("<Button-4>", self.on_zoom)
        self.canvas.bind("<Button-5>", self.on_zoom)
        
        # Binds Keyboard
        self.root.bind("<space>", lambda e: self.randomize_route())
        self.root.bind("<Return>", lambda e: self.toggle_animation())

    # --- FUNGSI KAMERA (Sama seperti sebelumnya) ---
    def on_pan_start(self, event):
        self.last_mouse_x, self.last_mouse_y = event.x, event.y

    def on_pan_drag(self, event):
        self.camera.offset_x += event.x - self.last_mouse_x
        self.camera.offset_y += event.y - self.last_mouse_y
        self.last_mouse_x, self.last_mouse_y = event.x, event.y
        if not self.is_animating: self.render()

    def on_zoom(self, event):
        zoom_factor = 1.1 if (event.num == 4 or getattr(event, 'delta', 0) > 0) else (1 / 1.1)
        cursor_x, cursor_y = event.x, event.y
        world_x, world_y = self.camera.screen_to_world(cursor_x, cursor_y)
        self.camera.scale = max(0.01, min(self.camera.scale * zoom_factor, 20.0))
        self.camera.offset_x = cursor_x - (world_x * self.camera.scale)
        self.camera.offset_y = cursor_y - (world_y * self.camera.scale)
        if not self.is_animating: self.render()

    def center_camera(self):
        self.camera.offset_x = self.canvas.winfo_width() / 2
        self.camera.offset_y = self.canvas.winfo_height() / 2

    # --- FUNGSI LOGIKA PERMAINAN ---
    def randomize_route(self):
        self.is_animating = False
        self.start_node, self.target_node = random.sample(range(self.map_generator.num_nodes), 2)
        
        path_nodes = self.map_generator.find_shortest_path(self.start_node, self.target_node)
        
        self.path_edges.clear()
        self.path_segments.clear()
        
        # Ekstrak data Bezier berurutan untuk animasi, perhatikan arah jalan!
        for i in range(len(path_nodes) - 1):
            u, v = path_nodes[i], path_nodes[i+1]
            self.path_edges.add((u, v)); self.path_edges.add((v, u))
            
            # Cari data kurva aslinya
            for edge in self.map_generator.edges_data:
                if edge['u'] == u and edge['v'] == v:
                    self.path_segments.append((edge['p0'], edge['p1'], edge['p2'], edge['p3']))
                    break
                elif edge['u'] == v and edge['v'] == u:
                    # Jika jalan dari V ke U, balik susunan titik kontrol agar mobil tidak mundur
                    self.path_segments.append((edge['p3'], edge['p2'], edge['p1'], edge['p0']))
                    break
                    
        # Reset state animasi
        self.current_segment_idx = 0
        self.current_t = 0.0
        self.render()

    def toggle_animation(self):
        """Memulai atau Menjeda Navigasi Kendaraan."""
        if not self.path_segments: return
        
        self.is_animating = not self.is_animating
        if self.is_animating:
            self.animate_step()

    def animate_step(self):
        """Loop Animasi menggunakan arc-length parameterization."""
        if not self.is_animating: return
        
        p0, p1, p2, p3 = self.path_segments[self.current_segment_idx]
        
        # 1. Hitung Turunan untuk Normalisasi Kecepatan
        dx, dy = BezierMath.calculate_derivative(p0, p1, p2, p3, self.current_t)
        magnitude = math.hypot(dx, dy)
        
        # 2. ARC-LENGTH PARAMETERIZATION: Delta t dinamis agar V fisik konstan
        if magnitude > 0:
            self.current_t += self.vehicle_speed / magnitude
        else:
            self.current_t += 0.01 
            
        # Pindah ke segmen kurva berikutnya jika t mencapai ujung
        if self.current_t >= 1.0:
            self.current_t = 0.0
            self.current_segment_idx += 1
            
            # Cek jika sampai tujuan
            if self.current_segment_idx >= len(self.path_segments):
                self.is_animating = False
                self.render()
                return

        self.render()
        # Jadwalkan frame berikutnya ~60 FPS (16ms)
        self.root.after(16, self.animate_step)

    # --- FUNGSI RENDER (Gambar) ---
    def render(self):
        self.canvas.delete("all")

        # 1. Gambar UI / Info
        self.canvas.create_text(20, 20, fill="white", anchor=tk.NW, font=("Arial", 12),
                                text="[SPASI] : Acak Rute | [ENTER] : Start/Pause Navigasi | [SCROLL] : Zoom")

        base_road_width = 30
        scaled_road_width = max(1, int(base_road_width * self.camera.scale))
        curve_res = 15 

        # 2. Render Jalan
        normal_edges = [e for e in self.map_generator.edges_data if (e['u'], e['v']) not in self.path_edges]
        highlighted_edges = [e for e in self.map_generator.edges_data if (e['u'], e['v']) in self.path_edges]

        for edges, color, extra_w in [(normal_edges, "#404050", 0), (highlighted_edges, "#f1fa8c", 4)]:
            w = scaled_road_width + max(0, int(extra_w * self.camera.scale))
            for edge in edges:
                points = []
                for i in range(curve_res + 1):
                    t = i / curve_res
                    wx, wy = BezierMath.calculate_point(edge['p0'], edge['p1'], edge['p2'], edge['p3'], t)
                    sx, sy = self.camera.world_to_screen(wx, wy)
                    points.extend([sx, sy])
                self.canvas.create_line(points, fill=color, width=w, capstyle=tk.ROUND, smooth=True)

        # 3. Render Simpul
        scaled_node_radius = max(2, int(20 * self.camera.scale))
        for idx, node in enumerate(self.map_generator.nodes):
            sx, sy = self.camera.world_to_screen(node['x'], node['y'])
            if idx == self.start_node: c, m = "#ff5555", 1.5
            elif idx == self.target_node: c, m = "#50fa7b", 1.5
            elif idx in [p for e in self.path_edges for p in e]: c, m = "#f1fa8c", 1.0
            else: c, m = "#7289da", 1.0
            r = scaled_node_radius * m
            self.canvas.create_oval(sx - r, sy - r, sx + r, sy + r, fill=c, outline="#2c2f33", width=2)

        # 4. Render Kendaraan Dinamis
        if self.path_segments and self.current_segment_idx < len(self.path_segments):
            self.render_vehicle()

    def render_vehicle(self):
        """Merotasi dan menggambar poligon kendaraan sesuai arah jalan."""
        p0, p1, p2, p3 = self.path_segments[self.current_segment_idx]
        
        # Posisi di World Space
        wx, wy = BezierMath.calculate_point(p0, p1, p2, p3, self.current_t)
        
        # Vektor arah menggunakan Turunan
        dx, dy = BezierMath.calculate_derivative(p0, p1, p2, p3, self.current_t)
        
        # Kalkulasi Sudut (Radian)
        angle = math.atan2(dy, dx)
        cos_a, sin_a = math.cos(angle), math.sin(angle)
        
        # Geometri Dasar Kendaraan (Panah / Pesawat kecil) di local origin (0,0) menghadap ke Kanan (0 derajat)
        # Akan di-scale sedikittt berdasarkan kamera agar terlihat proporsional tapi tidak hilang
        v_scale = max(1.0, self.camera.scale * 1.5)
        base_shape = [
            (25 * v_scale, 0),             # Hidung depan
            (-15 * v_scale, -15 * v_scale),# Sayap kiri belakang
            (-5 * v_scale, 0),             # Ekor tengah
            (-15 * v_scale, 15 * v_scale)  # Sayap kanan belakang
        ]
        
        # Translasi ke Screen Space
        cx, cy = self.camera.world_to_screen(wx, wy)
        
        rotated_poly = []
        for lx, ly in base_shape:
            # 1. Rotasi menggunakan Matriks Affine 2D
            rx = (lx * cos_a) - (ly * sin_a)
            ry = (lx * sin_a) + (ly * cos_a)
            # 2. Translasi ke posisi layar kamera
            rotated_poly.extend([cx + rx, cy + ry])
            
        # Gambar Kendaraan
        self.canvas.create_polygon(rotated_poly, fill="#ff79c6", outline="white", width=2)

if __name__ == "__main__":
    root = tk.Tk()
    app = VectorMapApp(root)
    root.mainloop()