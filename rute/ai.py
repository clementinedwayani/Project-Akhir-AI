import networkx as nx
import matplotlib
matplotlib.use('TkAgg')  # Set TkAgg backend for Windows
import matplotlib.pyplot as plt
import math
import heapq
from typing import List, Dict, Tuple, Optional, Set
from dataclasses import dataclass, field
from datetime import datetime, time
import re
import os

# Haversine formula to calculate distance between two points (lat, lon) in kilometers
def haversine(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    R = 6371.0  # Earth's radius in kilometers
    lat1, lon1, lat2, lon2 = map(math.radians, [lat1, lon1, lat2, lon2])
    dlat = lat2 - lat1
    dlon = lon2 - lon1
    a = math.sin(dlat / 2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon / 2)**2
    c = 2 * math.asin(math.sqrt(a))
    return R * c

# Calculate travel time in minutes given distance (km) and speed (km/h)
def calculate_travel_time(distance_km: float, speed_kmh: float = 30.0) -> float:
    return (distance_km / speed_kmh) * 60  # Convert hours to minutes

# Parse operational hours and check if attraction is open
def is_attraction_open(hours: str, current_time: datetime) -> bool:
    """Check if an attraction is open at the given time."""
    current_day = current_time.strftime('%A')[:3].lower()  # e.g., 'mon'
    current_time_only = current_time.time()
    
    # Handle "24 jam" case
    if hours.lower() == "24 jam":
        return True
    
    # Split multiple day ranges (e.g., "Tue-Thu 13:00-15:00, Fri-Sun 10:00-15:00")
    periods = hours.split(',')
    days_map = {
        'mon': 'monday', 'tue': 'tuesday', 'wed': 'wednesday',
        'thu': 'thursday', 'fri': 'friday', 'sat': 'saturday', 'sun': 'sunday'
    }
    
    for period in periods:
        period = period.strip()
        # Extract days and times
        day_match = re.match(r'(\w{3}(?:-\w{3})?)\s+(\d{2}:\d{2})-(\d{2}:\d{2})', period)
        if not day_match:
            # Handle special case for Kampung Wisata Batik Kauman
            if 'weekday' in period.lower():
                day_part, time_part = period.split('(')
                time_match = re.match(r'(\d{2}:\d{2})-(\d{2}:\d{2})', time_part.strip(')'))
                if time_match:
                    start, end = time_match.groups()
                    start = datetime.strptime(start, '%H:%M').time()
                    end = datetime.strptime(end, '%H:%M').time()
                    if current_day in ['mon', 'tue', 'wed', 'thu', 'fri']:
                        if start <= current_time_only <= end:
                            return True
            elif 'weekend' in period.lower():
                day_part, time_part = period.split('(')
                time_match = re.match(r'(\d{2}:\d{2})-(\d{2}:\d{2})', time_part.strip(')'))
                if time_match:
                    start, end = time_match.groups()
                    start = datetime.strptime(start, '%H:%M').time()
                    end = datetime.strptime(end, '%H:%M').time()
                    if current_day in ['sat', 'sun']:
                        if start <= current_time_only <= end:
                            return True
            continue
        days_str, start_time, end_time = day_match.groups()
        
        # Parse times
        try:
            start = datetime.strptime(start_time, '%H:%M').time()
            end = datetime.strptime(end_time, '%H:%M').time()
        except ValueError:
            continue
        
        # Handle day ranges (e.g., Tue-Thu)
        if '-' in days_str:
            start_day, end_day = days_str.split('-')
            start_idx = list(days_map.keys()).index(start_day.lower())
            end_idx = list(days_map.keys()).index(end_day.lower())
            if start_idx <= end_idx:
                valid_days = list(days_map.keys())[start_idx:end_idx + 1]
            else:
                valid_days = list(days_map.keys())[start_idx:] + list(days_map.keys())[:end_idx + 1]
        else:
            valid_days = [days_str.lower()]
        
        # Check if current day is in valid days
        if current_day in valid_days:
            # Check if current time is within hours
            if start <= current_time_only <= end or (end < start and (current_time_only >= start or current_time_only <= end)):
                return True
    
    return False

@dataclass
class Node:
    """Node for A* algorithm"""
    halte_id: str
    g_cost: float = float('inf')  # Cost from start
    h_cost: float = 0.0  # Heuristic cost to goal
    f_cost: float = field(init=False)  # Total cost
    parent: Optional['Node'] = None
    route_used: str = ""
    
    def __post_init__(self):
        self.f_cost = self.g_cost + self.h_cost
    
    def __lt__(self, other):
        return self.f_cost < other.f_cost

class BusRouteSystem:
    def __init__(self):
        # Bus stops (halte) data
        self.halte_data = [
            {"id": "H01", "name": "Jurug (Solo Safari)", "lat": -7.56513474408024, "lon": 110.858685876169, "routes": ["K1", "FD2"]},
            {"id": "H02", "name": "UNS", "lat": -7.56455236195493, "lon": 110.8561722718, "routes": ["K1"]},
            {"id": "H03", "name": "Vastenburg", "lat": -7.57175743180982, "lon": 110.8297470581, "routes": ["K1"]},
            {"id": "H04", "name": "Gladag", "lat": -7.57203341152584, "lon": 110.827739168751, "routes": ["K1"]},
            {"id": "H05", "name": "Pasar Pon Selatan", "lat": -7.57053451729552, "lon": 110.822770250784, "routes": ["K1"]},
            {"id": "H06", "name": "Sriwedari 1 Selatan", "lat": -7.56729936753916, "lon": 110.812057143543, "routes": ["K1", "FD8"]},
            {"id": "H07", "name": "Sriwedari 2 Selatan", "lat": -7.56795480396892, "lon": 110.814320671231, "routes": ["K1", "FD8"]},
            {"id": "H08", "name": "Colomadu Utara", "lat": -7.53253942382143, "lon": 110.748923594566, "routes": ["K1"]},
            {"id": "H09", "name": "Tugu Lilin", "lat": -7.567581219, "lon": 110.7835961, "routes": ["K3"]},
            {"id": "H10", "name": "Vestenburg (Kantor Pos)", "lat": -7.57140133360988, "lon": 110.829647652826, "routes": ["K3"]},
            {"id": "H11", "name": "Balai Kota", "lat": -7.56993057310902, "lon": 110.830046989632, "routes": ["K3", "FD2"]},
            {"id": "H12", "name": "Pasar Gede", "lat": -7.5684698254206, "lon": 110.831727375632, "routes": ["K3"]},
            {"id": "H13", "name": "Solo Techno Park", "lat": -7.55653679470118, "lon": 110.852209973442, "routes": ["K3"]},
            {"id": "H14", "name": "Kantor Kecamatan Jebres", "lat": -7.55541900174376, "lon": 110.854977935521, "routes": ["K3"]},
            {"id": "H15", "name": "Halte RS Jiwa / Taman Lansia", "lat": -7.55707042819373, "lon": 110.860610255225, "routes": ["K3"]},
            {"id": "H16", "name": "Halte Kecamatan Colomadu", "lat": -7.532634345, "lon": 110.7487969, "routes": ["K4"]},
            {"id": "H17", "name": "Stadion Manahan", "lat": -7.556663684, "lon": 110.8048193, "routes": ["K4"]},
            {"id": "H18", "name": "Terminal Tirtonadi", "lat": -7.551298569, "lon": 110.8182099, "routes": ["K4", "K6"]},
            {"id": "H19", "name": "Ngapeman", "lat": -7.568500872, "lon": 110.8166443, "routes": ["K5", "FD8"]},
            {"id": "H20", "name": "Sriwedari", "lat": -7.567047482, "lon": 110.8118452, "routes": ["K5"]},
            {"id": "H21", "name": "Landasan Udara (Pasar Colomadu)", "lat": -7.53171343, "lon": 110.7473448, "routes": ["K5", "FD7"]},
            {"id": "H22", "name": "Ngarsopuro", "lat": -7.569086355, "lon": 110.8221284, "routes": ["K6"]},
            {"id": "H23", "name": "Pasar Kembang", "lat": -7.571950677, "lon": 110.8166645, "routes": ["K6"]},
            {"id": "H24", "name": "Sriwedari 2 Utara", "lat": -7.567852615, "lon": 110.8146095, "routes": ["FD2", "FD8"]},
            {"id": "H25", "name": "Museum Keris B", "lat": -7.568829291, "lon": 110.8106188, "routes": ["FD8"]},
            {"id": "H26", "name": "Mangkunegaran", "lat": -7.567624751, "lon": 110.8220978, "routes": ["FD9"]},
            {"id": "H27", "name": "Sahid", "lat": -7.564166826, "lon": 110.8185673, "routes": ["FD8", "FD9"]},
            {"id": "H28", "name": "Pasar Klewer", "lat": -7.575037806, "lon": 110.8264383, "routes": ["FD10"]},
            {"id": "H29", "name": "Pasar Pucang Sawit A", "lat": -7.567996022, "lon": 110.8582507, "routes": ["FD10"]},
        ]
        
        # Tourist attractions (wisata) data with cost and operational hours
        self.wisata_data = [
            {"id": "W01", "name": "Solo Safari", "lat": -7.564391741, "lon": 110.8586613, "halte": ["H01"], "hours": "08:30 - 16:30", "cost": "weekday: Rp45,000 (child), Rp55,000 (adult); weekend: Rp60,000 (child), Rp75,000 (adult)"},
            {"id": "W02", "name": "Danau UNS", "lat": -7.561172246, "lon": 110.8581931, "halte": ["H02"], "hours": "24 jam", "cost": "Free"},
            {"id": "W03", "name": "Benteng Vastenburg", "lat": -7.571804006, "lon": 110.8307858, "halte": ["H03", "H10"], "hours": "24 jam", "cost": "Free"},
            {"id": "W04", "name": "Kampung Wisata Batik Kauman", "lat": -7.573215566, "lon": 110.8263633, "halte": ["H04"], "hours": "09:00 - 18:00 (weekday), 08:00 - 18:00 (weekend)", "cost": "Free"},
            {"id": "W05", "name": "Pasar Triwindu", "lat": -7.568984669, "lon": 110.8225384, "halte": ["H05"], "hours": "09:00 - 16:00", "cost": "Free"},
            {"id": "W06", "name": "Taman Sriwedari", "lat": -7.568224905, "lon": 110.8129629, "halte": ["H06", "H07", "H20"], "hours": "24 jam", "cost": "Free"},
            {"id": "W07", "name": "De Tjolomadoe", "lat": -7.533922576, "lon": 110.7498663, "halte": ["H08", "H16", "H21"], "hours": "09:00 - 17:00", "cost": "Rp40,000"},
            {"id": "W08", "name": "Lapangan Makamhaji", "lat": -7.5691203, "lon": 110.7831005, "halte": ["H09"], "hours": "24 jam", "cost": "Free"},
            {"id": "W09", "name": "Balaikota Surakarta", "lat": -7.569192352, "lon": 110.8296584, "halte": ["H11"], "hours": "24 jam", "cost": "Free"},
            {"id": "W10", "name": "Pasar Gede", "lat": -7.569143893, "lon": 110.8314553, "halte": ["H12"], "hours": "24 jam", "cost": "Free"},
            {"id": "W11", "name": "Solo Techno Park", "lat": -7.555835181, "lon": 110.8538009, "halte": ["H13"], "hours": "07:30 - 16:00", "cost": "Free"},
            {"id": "W12", "name": "Taman Cerdas", "lat": -7.553839457, "lon": 110.8534741, "halte": ["H14"], "hours": "09:00 - 21:00", "cost": "Free"},
            {"id": "W13", "name": "Taman Lansia", "lat": -7.55669203, "lon": 110.8607455, "halte": ["H15"], "hours": "24 jam", "cost": "Free"},
            {"id": "W14", "name": "Stadion Manahan", "lat": -7.555259829, "lon": 110.8065227, "halte": ["H17"], "hours": "05:30 - 21:00", "cost": "Free"},
            {"id": "W15", "name": "Taman Tirtonadi", "lat": -7.551283848, "lon": 110.8204733, "halte": ["H18"], "hours": "24 jam", "cost": "Free"},
            {"id": "W16", "name": "Tumurun Private Museum", "lat": -7.570257605, "lon": 110.8164116, "halte": ["H19", "H23"], "hours": "Tue-Thu 13:00-15:00, Fri-Sun 10:00-15:00", "cost": "Rp25,000"},
            {"id": "W17", "name": "Ngarsopuro Night Market", "lat": -7.568494751, "lon": 110.822291, "halte": ["H22"], "hours": "17:00 - 23:00", "cost": "Free"},
            {"id": "W18", "name": "Taman Balikota Solo", "lat": -7.569219287, "lon": 110.8298679, "halte": ["H11"], "hours": "24 jam", "cost": "Free"},
            {"id": "W19", "name": "Museum Radya Pustaka", "lat": -7.568292105, "lon": 110.8144969, "halte": ["H24"], "hours": "08:00 - 16:00", "cost": "Rp10,000 (general), Rp7,500 (student), Rp5,000 (Solo student)"},
            {"id": "W20", "name": "Pasar Malangjiwan Colomadu", "lat": -7.531636047, "lon": 110.7472482, "halte": ["H21"], "hours": "24 jam", "cost": "Free"},
            {"id": "W21", "name": "Museum Keris Nusantara", "lat": -7.568754681, "lon": 110.8107542, "halte": ["H25"], "hours": "08:00 - 16:00", "cost": "Rp10,000"},
            {"id": "W22", "name": "Loji Gandrung", "lat": -7.566305927, "lon": 110.8095326, "halte": ["H06"], "hours": "08:00 - 16:00", "cost": "Rp10,000"},
            {"id": "W23", "name": "Gedung Wayang Orang Dance Theatre", "lat": -7.56905024, "lon": 110.812558, "halte": ["H24", "H07"], "hours": "19:00 - 23:00", "cost": "Not specified"},
            {"id": "W24", "name": "House of Danar Hadi", "lat": -7.568506445, "lon": 110.8162107, "halte": ["H19"], "hours": "09:00 - 17:00", "cost": "Rp35,000 (general), Rp15,000 (student)"},
            {"id": "W25", "name": "Taman Punggawan Ngesus", "lat": -7.564517132, "lon": 110.818271, "halte": ["H27"], "hours": "24 jam", "cost": "Free"},
            {"id": "W26", "name": "Pura Mangkunegaran", "lat": -7.566613944, "lon": 110.8228758, "halte": ["H26"], "hours": "09:00 - 15:00", "cost": "Rp20,000"},
            {"id": "W27", "name": "Pasar Klewer", "lat": -7.575178766, "lon": 110.8267555, "halte": ["H28"], "hours": "24 jam", "cost": "Free"},
            {"id": "W28", "name": "Taman Sunan Jogo Kali", "lat": -7.569809858, "lon": 110.8581447, "halte": ["H29"], "hours": "06:00 - 21:00", "cost": "Free"},
        ]
        
        # Create adjacency graph based on route connections
        self.graph = self._build_graph()
        
        # Create halte lookup dictionary
        self.halte_dict = {h["id"]: h for h in self.halte_data}
        
        # Route colors for visualization
        self.route_colors = {
            "K1": "#FF6B6B",    # Red
            "K3": "#4ECDC4",    # Teal
            "K4": "#45B7D1",    # Blue
            "K5": "#96CEB4",    # Green
            "K6": "#FECA57",    # Yellow
            "FD2": "#FF9FF3",   # Pink
            "FD7": "#54A0FF",   # Light Blue
            "FD8": "#5F27CD",   # Purple
            "FD9": "#00D2D3",   # Cyan
            "FD10": "#FF9F43"   # Orange
        }
    
    def _build_graph(self) -> Dict[str, List[Tuple[str, float, str]]]:
        """Build adjacency graph with connections between haltes on same routes"""
        graph = {}
        for halte in self.halte_data:
            graph[halte["id"]] = []
        for i, halte1 in enumerate(self.halte_data):
            for j, halte2 in enumerate(self.halte_data):
                if i != j:
                    common_routes = set(halte1["routes"]) & set(halte2["routes"])
                    if common_routes:
                        distance = haversine(
                            halte1["lat"], halte1["lon"],
                            halte2["lat"], halte2["lon"]
                        )
                        route = list(common_routes)[0]
                        graph[halte1["id"]].append((halte2["id"], distance, route))
        return graph
    
    def visualize_route_graph(self, highlight_path: Optional[List[str]] = None, title_suffix: str = ""):
        """Visualize the BST route network as a graph with enhanced styling"""
        try:
            plt.figure(figsize=(16, 12))
            plt.clf()
            
            G = nx.Graph()
            
            # Add nodes (haltes) with positions based on lat/lon
            pos = {}
            for halte in self.halte_data:
                G.add_node(halte["id"], 
                          label=halte["name"], 
                          routes=halte["routes"])
                # Use lon as x and negative lat as y for geographic positioning
                pos[halte["id"]] = (halte["lon"], -halte["lat"])
            
            # Add edges (route connections) with proper attributes
            edge_routes = {}
            for halte_id, connections in self.graph.items():
                for neighbor_id, distance, route in connections:
                    edge_key = tuple(sorted([halte_id, neighbor_id]))
                    if edge_key not in edge_routes:
                        G.add_edge(halte_id, neighbor_id, weight=distance, route=route)
                        edge_routes[edge_key] = route
            
            # Create separate edge lists for each route for better visualization
            route_edges = {}
            for route in self.route_colors.keys():
                route_edges[route] = [(u, v) for u, v, d in G.edges(data=True) if d['route'] == route]
            
            # Draw edges by route with different colors
            for route, edges in route_edges.items():
                if edges:
                    # Highlight path edges if provided
                    if highlight_path:
                        path_edges = [(highlight_path[i], highlight_path[i+1]) for i in range(len(highlight_path)-1)]
                        path_edges_set = set(path_edges + [(v, u) for u, v in path_edges])
                        
                        # Separate highlighted and normal edges
                        highlighted_edges = [e for e in edges if e in path_edges_set or (e[1], e[0]) in path_edges_set]
                        normal_edges = [e for e in edges if e not in highlighted_edges]
                        
                        # Draw normal edges
                        if normal_edges:
                            nx.draw_networkx_edges(G, pos, edgelist=normal_edges, 
                                                 edge_color=self.route_colors[route], 
                                                 width=1.5, alpha=0.6)
                        
                        # Draw highlighted edges
                        if highlighted_edges:
                            nx.draw_networkx_edges(G, pos, edgelist=highlighted_edges, 
                                                 edge_color=self.route_colors[route], 
                                                 width=4, alpha=1.0)
                    else:
                        nx.draw_networkx_edges(G, pos, edgelist=edges, 
                                             edge_color=self.route_colors[route], 
                                             width=2, alpha=0.7)
            
            # Draw nodes with different colors for highlights
            if highlight_path:
                # Normal nodes
                normal_nodes = [n for n in G.nodes if n not in highlight_path]
                if normal_nodes:
                    nx.draw_networkx_nodes(G, pos, nodelist=normal_nodes, 
                                         node_color='lightblue', 
                                         node_size=300, alpha=0.8)
                
                # Highlighted nodes
                if len(highlight_path) > 0:
                    # Start node (green)
                    nx.draw_networkx_nodes(G, pos, nodelist=[highlight_path[0]], 
                                         node_color='lightgreen', 
                                         node_size=500, alpha=1.0)
                    
                    # End node (red)
                    if len(highlight_path) > 1:
                        nx.draw_networkx_nodes(G, pos, nodelist=[highlight_path[-1]], 
                                             node_color='lightcoral', 
                                             node_size=500, alpha=1.0)
                    
                    # Intermediate nodes (yellow)
                    if len(highlight_path) > 2:
                        nx.draw_networkx_nodes(G, pos, nodelist=highlight_path[1:-1], 
                                             node_color='lightyellow', 
                                             node_size=400, alpha=1.0)
            else:
                nx.draw_networkx_nodes(G, pos, node_color='lightblue', 
                                     node_size=300, alpha=0.8)
            
            # Add node labels with better positioning
            labels = {node: data['label'].replace(' ', '\n') if len(data['label']) > 15 
                     else data['label'] for node, data in G.nodes(data=True)}
            nx.draw_networkx_labels(G, pos, labels=labels, font_size=8, 
                                  font_weight='bold', bbox=dict(boxstyle="round,pad=0.2", 
                                  facecolor="white", alpha=0.8))
            
            # Create legend for routes
            legend_elements = []
            for route, color in self.route_colors.items():
                if any(route in halte["routes"] for halte in self.halte_data):
                    legend_elements.append(plt.Line2D([0], [0], color=color, lw=3, label=f'Rute {route}'))
            
            plt.legend(handles=legend_elements, loc='upper left', bbox_to_anchor=(0, 1))
            
            # Set title based on context
            if highlight_path:
                title = f"Jaringan Bus Solo Trans - Rute Terpilih {title_suffix}"
                plt.suptitle(title, fontsize=16, fontweight='bold')
                plt.title(f"Dari: {self.halte_dict[highlight_path[0]]['name']} → Ke: {self.halte_dict[highlight_path[-1]]['name']}", 
                         fontsize=12, pad=20)
            else:
                title = f"Jaringan Rute Bus Solo Trans (BST) {title_suffix}"
                plt.title(title, fontsize=16, fontweight='bold', pad=20)
            
            plt.axis('equal')
            plt.axis('off')
            plt.tight_layout()
            
            # Try to show the plot
            try:
                plt.show(block=False)
                plt.pause(0.1)  # Small pause to ensure plot is displayed
            except Exception as e:
                print(f"⚠ Tidak dapat menampilkan grafik secara interaktif: {e}")
                output_file = f"route_graph_{title_suffix.replace(' ', '_').lower()}.png"
                plt.savefig(output_file, dpi=300, bbox_inches='tight')
                print(f"✅ Grafik telah disimpan sebagai '{output_file}' di direktori saat ini.")
        
        except Exception as e:
            print(f"❌ Error saat membuat visualisasi grafik: {e}")
            import traceback
            traceback.print_exc()
    
    def heuristic(self, halte1_id: str, halte2_id: str) -> float:
        """Heuristic function: straight-line distance between two haltes"""
        halte1 = self.halte_dict[halte1_id]
        halte2 = self.halte_dict[halte2_id]
        return haversine(halte1["lat"], halte1["lon"], halte2["lat"], halte2["lon"])
    
    def a_star(self, start_id: str, goal_id: str) -> Optional[Dict]:
        """A* pathfinding algorithm to find optimal route"""
        if start_id not in self.halte_dict or goal_id not in self.halte_dict:
            return None
        if start_id == goal_id:
            return {
                "path": [start_id],
                "total_distance": 0.0,
                "total_time": 0.0,
                "routes": [],
                "transfers": 0
            }
        open_set = []
        heapq.heappush(open_set, Node(start_id, g_cost=0.0, h_cost=self.heuristic(start_id, goal_id)))
        closed_set: Set[str] = set()
        open_set_dict = {start_id: 0.0}
        came_from = {}
        g_score = {start_id: 0.0}
        
        while open_set:
            current_node = heapq.heappop(open_set)
            current_id = current_node.halte_id
            if current_id in closed_set:
                continue
            if current_id == goal_id:
                return self._reconstruct_path(came_from, start_id, goal_id, g_score[goal_id])
            closed_set.add(current_id)
            for neighbor_id, distance, route in self.graph.get(current_id, []):
                if neighbor_id in closed_set:
                    continue
                tentative_g_score = g_score[current_id] + distance
                if neighbor_id not in open_set_dict or tentative_g_score < open_set_dict[neighbor_id]:
                    came_from[neighbor_id] = (current_id, route)
                    g_score[neighbor_id] = tentative_g_score
                    h_score = self.heuristic(neighbor_id, goal_id)
                    neighbor_node = Node(neighbor_id, g_cost=tentative_g_score, h_cost=h_score)
                    heapq.heappush(open_set, neighbor_node)
                    open_set_dict[neighbor_id] = tentative_g_score
        
        return None  # No path found
    
    def _reconstruct_path(self, came_from: Dict, start_id: str, goal_id: str, total_distance: float) -> Dict:
        """Reconstruct path from A* algorithm results"""
        path = []
        routes = []
        current = goal_id
        
        while current != start_id:
            path.append(current)
            parent, route = came_from[current]
            routes.append(route)
            current = parent
        
        path.append(start_id)
        path.reverse()
        routes.reverse()
        
        # Calculate transfers
        transfers = 0
        for i in range(1, len(routes)):
            if routes[i] != routes[i-1]:
                transfers += 1
        
        total_time = calculate_travel_time(total_distance)
        
        return {
            "path": path,
            "total_distance": total_distance,
            "total_time": total_time,
            "routes": routes,
            "transfers": transfers
        }
    
    def find_nearest_halte(self, lat: float, lon: float) -> str:
        """Find nearest halte to given coordinates"""
        min_distance = float('inf')
        nearest_halte = None
        
        for halte in self.halte_data:
            distance = haversine(lat, lon, halte["lat"], halte["lon"])
            if distance < min_distance:
                min_distance = distance
                nearest_halte = halte["id"]
        
        return nearest_halte
    
    def get_wisata_by_halte(self, halte_id: str) -> List[Dict]:
        """Get tourist attractions accessible from a specific halte"""
        wisata_list = []
        for wisata in self.wisata_data:
            if halte_id in wisata["halte"]:
                wisata_list.append(wisata)
        return wisata_list
    
    def plan_trip(self, start_coords: Tuple[float, float], target_wisata: List[str], 
                  current_time: datetime, budget: float = float('inf')) -> Dict:
        """Plan a trip to multiple tourist attractions"""
        start_lat, start_lon = start_coords
        start_halte = self.find_nearest_halte(start_lat, start_lon)
        
        # Filter wisata based on opening hours and budget
        available_wisata = []
        for wisata_id in target_wisata:
            wisata = next((w for w in self.wisata_data if w["id"] == wisata_id), None)
            if wisata and is_attraction_open(wisata["hours"], current_time):
                available_wisata.append(wisata)
        
        if not available_wisata:
            return {"error": "Tidak ada wisata yang tersedia pada waktu ini"}
        
        # Find optimal route visiting all attractions
        trip_plan = {
            "start_halte": start_halte,
            "start_location": self.halte_dict[start_halte]["name"],
            "attractions": [],
            "total_distance": 0.0,
            "total_time": 0.0,
            "total_cost": 0.0,
            "total_transfers": 0
        }
        
        current_halte = start_halte
        visited_wisata = []
        
        for wisata in available_wisata:
            # Find nearest halte to this attraction
            nearest_halte = min(wisata["halte"], 
                              key=lambda h: haversine(
                                  self.halte_dict[h]["lat"], self.halte_dict[h]["lon"],
                                  wisata["lat"], wisata["lon"]
                              ))
            
            # Find route from current location to attraction
            route_result = self.a_star(current_halte, nearest_halte)
            
            if route_result:
                # Parse cost information
                cost_info = self._parse_cost(wisata["cost"], current_time)
                
                attraction_info = {
                    "wisata": wisata,
                    "halte": nearest_halte,
                    "route": route_result,
                    "cost": cost_info
                }
                
                trip_plan["attractions"].append(attraction_info)
                trip_plan["total_distance"] += route_result["total_distance"]
                trip_plan["total_time"] += route_result["total_time"]
                trip_plan["total_cost"] += cost_info["amount"]
                trip_plan["total_transfers"] += route_result["transfers"]
                
                current_halte = nearest_halte
                visited_wisata.append(wisata)
        
        return trip_plan
    
    def _parse_cost(self, cost_str: str, current_time: datetime) -> Dict:
        """Parse cost string and return appropriate cost for current time"""
        if cost_str.lower() == "free":
            return {"amount": 0, "description": "Gratis"}
        
        if "not specified" in cost_str.lower():
            return {"amount": 0, "description": "Biaya tidak disebutkan"}
        
        # Check if it's weekend or weekday
        is_weekend = current_time.weekday() >= 5  # Saturday = 5, Sunday = 6
        
        # Extract numerical values
        import re
        numbers = re.findall(r'Rp([\d,]+)', cost_str)
        
        if not numbers:
            return {"amount": 0, "description": cost_str}
        
        # Convert to integers
        amounts = [int(n.replace(',', '')) for n in numbers]
        
        if "weekend" in cost_str.lower() and "weekday" in cost_str.lower():
            # Has different prices for weekday/weekend
            if is_weekend:
                # Usually weekend prices are higher, take the higher values
                amount = max(amounts) if amounts else 0
                return {"amount": amount, "description": f"Weekend: Rp{amount:,}"}
            else:
                # Weekday prices are usually lower
                amount = min(amounts) if amounts else 0
                return {"amount": amount, "description": f"Weekday: Rp{amount:,}"}
        else:
            # Single price or adult price
            amount = max(amounts) if amounts else 0
            return {"amount": amount, "description": f"Rp{amount:,}"}
    
    def print_trip_plan(self, trip_plan: Dict):
        """Print formatted trip plan"""
        if "error" in trip_plan:
            print(f"❌ {trip_plan['error']}")
            return
        
        print("🗺️  RENCANA PERJALANAN WISATA SOLO")
        print("=" * 50)
        print(f"📍 Titik Awal: {trip_plan['start_location']} ({trip_plan['start_halte']})")
        print(f"🎯 Jumlah Destinasi: {len(trip_plan['attractions'])}")
        print(f"📏 Total Jarak: {trip_plan['total_distance']:.2f} km")
        print(f"⏱️  Total Waktu Perjalanan: {trip_plan['total_time']:.1f} menit")
        print(f"💰 Total Biaya Wisata: Rp{trip_plan['total_cost']:,.0f}")
        print(f"🔄 Total Transfer: {trip_plan['total_transfers']}")
        print()
        
        for i, attraction in enumerate(trip_plan['attractions'], 1):
            wisata = attraction['wisata']
            route = attraction['route']
            cost = attraction['cost']
            
            print(f"🎯 DESTINASI {i}: {wisata['name']}")
            print(f"   📍 Halte Terdekat: {self.halte_dict[attraction['halte']]['name']}")
            print(f"   🕒 Jam Operasional: {wisata['hours']}")
            print(f"   💰 Biaya: {cost['description']}")
            
            if len(route['path']) > 1:
                print(f"   🚌 Rute Bus:")
                for j in range(len(route['path']) - 1):
                    from_halte = self.halte_dict[route['path'][j]]['name']
                    to_halte = self.halte_dict[route['path'][j + 1]]['name']
                    route_name = route['routes'][j]
                    print(f"      {from_halte} → {to_halte} (Rute {route_name})")
                
                print(f"   📏 Jarak: {route['total_distance']:.2f} km")
                print(f"   ⏱️  Waktu: {route['total_time']:.1f} menit")
                print(f"   🔄 Transfer: {route['transfers']}")
            else:
                print("   🚶 Sudah berada di lokasi")
            
            print()
    
    def get_all_routes_info(self) -> Dict:
        """Get information about all available bus routes"""
        routes_info = {}
        
        for halte in self.halte_data:
            for route in halte["routes"]:
                if route not in routes_info:
                    routes_info[route] = {
                        "haltes": [],
                        "color": self.route_colors.get(route, "#000000")
                    }
                routes_info[route]["haltes"].append({
                    "id": halte["id"],
                    "name": halte["name"],
                    "lat": halte["lat"],
                    "lon": halte["lon"]
                })
        
        return routes_info
    
    def find_attractions_near_route(self, route_name: str, current_time: datetime) -> List[Dict]:
        """Find attractions accessible via a specific route"""
        route_haltes = []
        for halte in self.halte_data:
            if route_name in halte["routes"]:
                route_haltes.append(halte["id"])
        
        accessible_attractions = []
        for wisata in self.wisata_data:
            # Check if any of the attraction's haltes are on this route
            if any(halte_id in route_haltes for halte_id in wisata["halte"]):
                if is_attraction_open(wisata["hours"], current_time):
                    accessible_attractions.append({
                        **wisata,
                        "cost_info": self._parse_cost(wisata["cost"], current_time)
                    })
        
        return accessible_attractions

# Example usage and testing
if __name__ == "__main__":
    # Initialize the system
    bst = BusRouteSystem()
    
    # Example 1: Plan a trip
    print("🚌 SISTEM PERENCANAAN WISATA BUS SOLO TRANS (BST)")
    print("=" * 60)
    
    # Starting point (example: near UNS)
    start_coordinates = (-7.5645, 110.8562)  # Near UNS
    
    # Target attractions
    target_attractions = ["W01", "W03", "W06", "W10"]  # Solo Safari, Benteng Vastenburg, Taman Sriwedari, Pasar Gede
    
    # Current time (example: Saturday 10:00 AM)
    from datetime import datetime
    current_time = datetime(2024, 6, 15, 10, 0)  # Saturday 10:00 AM
    
    # Plan the trip
    trip_plan = bst.plan_trip(start_coordinates, target_attractions, current_time)
    
    # Print the plan
    bst.print_trip_plan(trip_plan)
    
    # Example 2: Visualize the network
    print("\n📊 VISUALISASI JARINGAN RUTE")
    print("=" * 30)
    
    try:
        # Show complete network
        bst.visualize_route_graph(title_suffix="- Jaringan Lengkap")
        
        # If there are attractions in the trip plan, highlight the routes
        if "attractions" in trip_plan and trip_plan["attractions"]:
            first_attraction = trip_plan["attractions"][0]
            if "route" in first_attraction and len(first_attraction["route"]["path"]) > 1:
                bst.visualize_route_graph(
                    highlight_path=first_attraction["route"]["path"],
                    title_suffix="- Rute ke Destinasi Pertama"
                )
    except Exception as e:
        print(f"⚠️  Visualisasi grafik tidak tersedia: {e}")
    
    # Example 3: Find attractions accessible by specific route
    print("\n🎯 WISATA YANG DAPAT DIAKSES MELALUI RUTE K1")
    print("=" * 50)
    
    k1_attractions = bst.find_attractions_near_route("K1", current_time)
    for i, attraction in enumerate(k1_attractions, 1):
        print(f"{i}. {attraction['name']}")
        print(f"   💰 {attraction['cost_info']['description']}")
        print(f"   🕒 {attraction['hours']}")
        print()
    
    # Example 4: Show all routes information
    print("\n📋 INFORMASI SEMUA RUTE BUS")
    print("=" * 30)
    
    routes_info = bst.get_all_routes_info()
    for route_name, info in routes_info.items():
        print(f"🚌 Rute {route_name}: {len(info['haltes'])} halte")
    
    print("\n✅ Sistem berhasil dijalankan!")
    print("\n💡 Tips:")
    print("   - Gunakan method plan_trip() untuk merencanakan perjalanan")
    print("   - Method visualize_route_graph() untuk melihat peta rute")
    print("   - Method find_attractions_near_route() untuk mencari wisata di rute tertentu")
    print("   - Semua jam operasional dan biaya sudah diperhitungkan otomatis")