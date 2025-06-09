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
        """Reconstruct the optimal path from A* results"""
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
        transfers = sum(1 for i in range(1, len(routes)) if routes[i] != routes[i-1])
        return {
            "path": path,
            "path_names": [self.halte_dict[h_id]["name"] for h_id in path],
            "total_distance": total_distance,
            "total_time": calculate_travel_time(total_distance),
            "routes": routes,
            "segment_distances": [distance for _, _, distance in [(came_from[p][0], came_from[p][1], haversine(
                self.halte_dict[came_from[p][0]]["lat"], self.halte_dict[came_from[p][0]]["lon"],
                self.halte_dict[p]["lat"], self.halte_dict[p]["lon"])) for p in path[1:]]],
            "transfers": transfers
        }
    
    def find_route(self, start_id: str, end_id: str) -> Optional[Dict]:
        """Find optimal route between two haltes using A*"""
        return self.a_star(start_id, end_id)
    
    def find_nearest_wisata(self, halte_id: str) -> Optional[Tuple[str, str, float]]:
        """Find the nearest tourist attraction to a given halte"""
        if halte_id not in self.halte_dict:
            return None
        halte = self.halte_dict[halte_id]
        min_distance = float('inf')
        nearest_wisata = None
        nearest_wisata_id = None
        for wisata in self.wisata_data:
            distance = haversine(halte["lat"], halte["lon"], wisata["lat"], wisata["lon"])
            if distance < min_distance:
                min_distance = distance
                nearest_wisata = wisata["name"]
                nearest_wisata_id = wisata["id"]
        return nearest_wisata_id, nearest_wisata, min_distance
    
    def get_route_to_attraction(self, start_id: str, attraction_name: str) -> Optional[Dict]:
        """Find route to a specific tourist attraction"""
        attraction = next((w for w in self.wisata_data if w["name"].lower() == attraction_name.lower()), None)
        if not attraction:
            return None
        best_halte = None
        min_distance = float('inf')
        for halte_id in attraction["halte"]:
            if halte_id in self.halte_dict:
                halte = self.halte_dict[halte_id]
                distance = haversine(halte["lat"], halte["lon"], attraction["lat"], attraction["lon"])
                if distance < min_distance:
                    min_distance = distance
                    best_halte = halte_id
        if not best_halte:
            return None
        route_result = self.find_route(start_id, best_halte)
        if route_result:
            route_result["destination_attraction"] = attraction["name"]
            route_result["walking_distance_to_attraction"] = min_distance
            route_result["attraction_hours"] = attraction["hours"]
            route_result["attraction_cost"] = attraction["cost"]
        return route_result
    
    def get_attractions_along_route(self, path: List[str], radius_km: float = 1.0) -> List[Dict]:
        """Find tourist attractions along the route within specified radius"""
        attractions_found = []
        for halte_id in path:
            halte = self.halte_dict[halte_id]
            for wisata in self.wisata_data:
                distance = haversine(halte["lat"], halte["lon"], wisata["lat"], wisata["lon"])
                if distance <= radius_km:
                    attractions_found.append({
                        "attraction": wisata["name"],
                        "attraction_id": wisata["id"],
                        "near_halte": halte["name"],
                        "near_halte_id": halte_id,
                        "distance_km": distance,
                        "walking_time_min": distance * 12,
                        "hours": wisata["hours"],
                        "cost": wisata["cost"]
                    })
        seen = set()
        unique_attractions = [attr for attr in attractions_found if not (attr["attraction_id"] in seen or seen.add(attr["attraction_id"]))]
        return sorted(unique_attractions, key=lambda x: x["distance_km"])
    
    def get_route_analysis(self, route_result: Dict) -> Dict:
        """Analyze route and provide recommendations"""
        if not route_result:
            return {}
        analysis = {
            "efficiency": "Unknown",
            "complexity": "Unknown",
            "cost_estimate": 0,
            "recommendations": [],
            "considerations": []
        }
        total_time = route_result["total_time"]
        if total_time <= 15:
            analysis["efficiency"] = "Sangat Efisien"
        elif total_time <= 30:
            analysis["efficiency"] = "Efisien"
        elif total_time <= 45:
            analysis["efficiency"] = "Sedang"
        else:
            analysis["efficiency"] = "Kurang Efisien"
        transfers = route_result["transfers"]
        if transfers == 0:
            analysis["complexity"] = "Sangat Mudah (Langsung)"
        elif transfers == 1:
            analysis["complexity"] = "Mudah (1 Transfer)"
        elif transfers == 2:
            analysis["complexity"] = "Sedang (2 Transfer)"
        else:
            analysis["complexity"] = "Rumit (3+ Transfer)"
        analysis["cost_estimate"] = (transfers + 1) * 3700
        if transfers == 0:
            analysis["recommendations"].append("✓ Rute langsung - sangat mudah dan efisien")
        else:
            analysis["recommendations"].append(f"⚠ Memerlukan {transfers} kali transfer")
        if total_time > 45:
            analysis["recommendations"].append("⚠ Perjalanan cukup lama, siapkan waktu ekstra")
        if route_result["total_distance"] > 15:
            analysis["recommendations"].append("⚠ Jarak cukup jauh, pastikan kondisi fisik prima")
        analysis["considerations"].extend([
            f"Estimasi biaya transportasi: Rp {analysis['cost_estimate']:,}",
            f"Waktu perjalanan: ~{total_time:.0f} menit",
            f"Jarak total: {route_result['total_distance']:.1f} km"
        ])
        if transfers > 0:
            analysis["considerations"].append(f"Waktu tunggu transfer: ~{transfers * 5}-{transfers * 10} menit tambahan")
        return analysis

def display_halte_list(bus_system):
    """Display all available haltes"""
    print("\n=== DAFTAR HALTE TERSEDIA ===")
    print("=" * 60)
    for halte in sorted(bus_system.halte_data, key=lambda x: x["id"]):
        routes_str = ", ".join(halte["routes"])
        print(f"{halte['id']}: {halte['name']} (Rute: {routes_str})")

def display_attraction_list(bus_system):
    """Display all available tourist attractions"""
    print("\n=== DAFTAR TEMPAT WISATA ===")
    print("=" * 60)
    for wisata in sorted(bus_system.wisata_data, key=lambda x: x["id"]):
        halte_str = ", ".join(wisata["halte"])
        print(f"{wisata['id']}: {wisata['name']} (Dekat halte: {halte_str})")
        print(f"  Jam Operasional: {wisata['hours']}")
        print(f"  Biaya Masuk: {wisata['cost']}")

def search_halte(bus_system, query):
    """Search for halte by name or ID"""
    query = query.lower()
    return [halte for halte in bus_system.halte_data if query in halte["id"].lower() or query in halte["name"].lower()]

def interactive_route_planner():
    """Interactive route planning system"""
    bus_system = BusRouteSystem()
    
    print("🚌 === SISTEM PERENCANAAN RUTE BUS SOLO === 🚌")
    print("Menggunakan Algoritma A* untuk rute optimal\n")
    
    while True:
        print("\n" + "="*60)
        print("MENU UTAMA:")
        print("1. Cari Rute Antar Halte")
        print("2. Cari Rute ke Tempat Wisata")
        print("3. Lihat Daftar Halte")
        print("4. Lihat Daftar Tempat Wisata")
        print("5. Cari Halte")
        print("6. Visualisasi Jaringan Rute")
        print("0. Keluar")
        print("="*60)
        
        choice = input("Pilih menu (0-6): ").strip()
        
        if choice == "0":
            print("Terima kasih telah menggunakan sistem ini! 🙏")
            break
        
        elif choice == "1":
            print("\n🗺️  PENCARIAN RUTE ANTAR HALTE")
            print("-" * 40)
            
            start_query = input("Dari halte (ID atau nama): ").strip()
            start_matches = search_halte(bus_system, start_query)
            
            if not start_matches:
                print("❌ Halte asal tidak ditemukan!")
                continue
            elif len(start_matches) > 1:
                print("Beberapa halte ditemukan:")
                for i, halte in enumerate(start_matches):
                    print(f"  {i+1}. {halte['id']}: {halte['name']}")
                try:
                    idx = int(input("Pilih nomor: ")) - 1
                    start_halte = start_matches[idx]
                except (ValueError, IndexError):
                    print("❌ Pilihan tidak valid!")
                    continue
            else:
                start_halte = start_matches[0]
            
            end_query = input("Ke halte (ID atau nama): ").strip()
            end_matches = search_halte(bus_system, end_query)
            
            if not end_matches:
                print("❌ Halte tujuan tidak ditemukan!")
                continue
            elif len(end_matches) > 1:
                print("Beberapa halte ditemukan:")
                for i, halte in enumerate(end_matches):
                    print(f"  {i+1}. {halte['id']}: {halte['name']}")
                try:
                    idx = int(input("Pilih nomor: ")) - 1
                    end_halte = end_matches[idx]
                except (ValueError, IndexError):
                    print("❌ Pilihan tidak valid!")
                    continue
            else:
                end_halte = end_matches[0]
            
            result = bus_system.find_route(start_halte["id"], end_halte["id"])
            
            if result:
                print(f"\n🎯 RUTE DITEMUKAN!")
                print("=" * 50)
                print(f"Dari: {result['path_names'][0]}")
                print(f"Ke: {result['path_names'][-1]}")
                print(f"Jarak Total: {result['total_distance']:.1f} km")
                print(f"Waktu Tempuh: ~{result['total_time']:.0f} menit")
                print(f"Jumlah Transfer: {result['transfers']}")
                
                print(f"\n📍 RUTE DETAIL:")
                for i, (halte_name, route) in enumerate(zip(result['path_names'], result['routes'] + [''])):
                    if i == len(result['path_names']) - 1:
                        print(f"  {i+1}. {halte_name} (TUJUAN)")
                    else:
                        print(f"  {i+1}. {halte_name} → Naik Bus {route}")
                
                analysis = bus_system.get_route_analysis(result)
                print(f"\n📊 ANALISIS RUTE:")
                print(f"Efisiensi: {analysis['efficiency']}")
                print(f"Kompleksitas: {analysis['complexity']}")
                print(f"Estimasi Biaya: Rp {analysis['cost_estimate']:,}")
                
                print(f"\n💡 REKOMENDASI:")
                for rec in analysis['recommendations']:
                    print(f"  {rec}")
                
                print(f"\n📋 PERTIMBANGAN:")
                for cons in analysis['considerations']:
                    print(f"  • {cons}")
                
                attractions = bus_system.get_attractions_along_route(result['path'])
                if attractions:
                    print(f"\n🏛️  TEMPAT WISATA DI SEPANJANG RUTE:")
                    for attr in attractions[:5]:
                        print(f"  • {attr['attraction']} (dekat {attr['near_halte']})")
                        print(f"    Jarak jalan kaki: {attr['distance_km']:.1f} km (~{attr['walking_time_min']:.0f} menit)")
                        print(f"    Jam Operasional: {attr['hours']}")
                        print(f"    Biaya Masuk: {attr['cost']}")
                
                # Visualize the selected route
                print("\n📊 Menampilkan visualisasi rute terpilih...")
                bus_system.visualize_route_graph(highlight_path=result['path'], title_suffix=f"- Dari {result['path_names'][0]} ke {result['path_names'][-1]}")
            else:
                print("❌ Tidak ada rute yang ditemukan!")
        
        elif choice == "2":
            print("\n🏛️  PENCARIAN RUTE KE TEMPAT WISATA")
            print("-" * 40)
            print("Tempat wisata populer:")
            popular = ["Solo Safari", "Benteng Vastenburg", "Pasar Gede", "Taman Sriwedari", "Pasar Klewer"]
            for i, attraction in enumerate(popular, 1):
                print(f"  {i}. {attraction}")
            
            attraction_name = input("Nama tempat wisata: ").strip()
            start_query = input("Dari halte (ID atau nama): ").strip()
            start_matches = search_halte(bus_system, start_query)
            
            if not start_matches:
                print("❌ Halte asal tidak ditemukan!")
                continue
            elif len(start_matches) > 1:
                print("Beberapa halte ditemukan:")
                for i, halte in enumerate(start_matches):
                    print(f"  {i+1}. {halte['id']}: {halte['name']}")
                try:
                    idx = int(input("Pilih nomor: ")) - 1
                    start_halte = start_matches[idx]
                except (ValueError, IndexError):
                    print("❌ Pilihan tidak valid!")
                    continue
            else:
                start_halte = start_matches[0]
            
            result = bus_system.get_route_to_attraction(start_halte["id"], attraction_name)
            
            if result:
                print(f"\n🎯 RUTE KE {result['destination_attraction']} DITEMUKAN!")
                print("=" * 50)
                print(f"Dari: {result['path_names'][0]}")
                print(f"Ke Halte: {result['path_names'][-1]}")
                print(f"Jarak Bus: {result['total_distance']:.1f} km")
                print(f"Waktu Bus: ~{result['total_time']:.0f} menit")
                print(f"Jarak Jalan Kaki: {result['walking_distance_to_attraction']:.1f} km")
                print(f"Waktu Jalan Kaki: ~{result['walking_distance_to_attraction']*12:.0f} menit")
                print(f"Transfer: {result['transfers']} kali")
                print(f"Jam Operasional: {result['attraction_hours']}")
                print(f"Biaya Masuk: {result['attraction_cost']}")
                
                print(f"\n📍 RUTE DETAIL:")
                for i, (halte_name, route) in enumerate(zip(result['path_names'], result['routes'] + [''])):
                    if i == len(result['path_names']) - 1:
                        print(f"  {i+1}. {halte_name} → Jalan kaki ke {result['destination_attraction']}")
                    else:
                        print(f"  {i+1}. {halte_name} → Naik Bus {route}")
                
                analysis = bus_system.get_route_analysis(result)
                print(f"\n📊 ANALISIS:")
                print(f"Efisiensi: {analysis['efficiency']}")
                print(f"Kompleksitas: {analysis['complexity']}")
                total_cost = analysis['cost_estimate']
                total_time = result['total_time'] + (result['walking_distance_to_attraction'] * 12)
                print(f"Total Biaya Transportasi: Rp {total_cost:,}")
                print(f"Total Waktu: ~{total_time:.0f} menit")
                
                # Visualize the selected route
                print("\n📊 Menampilkan visualisasi rute terpilih...")
                bus_system.visualize_route_graph(highlight_path=result['path'], title_suffix=f"- Ke {result['destination_attraction']}")
            else:
                print("❌ Rute ke tempat wisata tidak ditemukan!")
        
        elif choice == "3":
            display_halte_list(bus_system)
        
        elif choice == "4":
            display_attraction_list(bus_system)
        
        elif choice == "5":
            print("\n🔍 PENCARIAN HALTE")
            print("-" * 40)
            query = input("Masukkan nama atau ID halte: ").strip()
            matches = search_halte(bus_system, query)
            if matches:
                print(f"\nDitemukan {len(matches)} halte:")
                for halte in matches:
                    routes_str = ", ".join(halte["routes"])
                    print(f"  • {halte['id']}: {halte['name']} (Rute: {routes_str})")
            else:
                print("❌ Tidak ada halte yang ditemukan!")
        
        elif choice == "6":
            print("\n📊 VISUALISASI JARINGAN RUTE")
            print("-" * 40)
            print("Menampilkan jaringan rute Bus Solo Trans...")
            bus_system.visualize_route_graph()
        
        else:
            print("❌ Pilihan tidak valid!")

def main():
    interactive_route_planner()

if __name__ == "__main__":
    main()
