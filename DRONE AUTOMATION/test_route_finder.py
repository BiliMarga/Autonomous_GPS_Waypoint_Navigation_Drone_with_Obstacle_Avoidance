import math
import shutup
shutup.please()

import networkx as nx
import osmnx as ox
import folium

def haversine_distance(lat1, lon1, lat2, lon2):
    R = 6371000.0  # Earth radius in meters
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_phi = math.radians(lat2 - lat1)
    delta_lambda = math.radians(lon2 - lon1)
    a = math.sin(delta_phi / 2.0)**2 + math.cos(phi1) * math.cos(phi2) * math.sin(delta_lambda / 2.0)**2
    c = 2.0 * math.atan2(math.sqrt(a), math.sqrt(1.0 - a))
    return R * c

def calculate_bearing(lat1, lon1, lat2, lon2):
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lambda = math.radians(lon2 - lon1)
    y = math.sin(delta_lambda) * math.cos(phi2)
    x = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lambda)
    bearing = math.degrees(math.atan2(y, x))
    return (bearing + 360.0) % 360.0

def bearing_to_cardinal(bearing):
    dirs = ["N", "NNE", "NE", "ENE", "E", "ESE", "SE", "SSE",
            "S", "SSW", "SW", "WSW", "W", "WNW", "NW", "NNW"]
    ix = round(bearing / (360.0 / len(dirs))) % len(dirs)
    return dirs[ix]

def get_coordinate(prompt, default_val=None):
    while True:
        try:
            val_str = input(f"{prompt} (default {default_val}): ").strip() if default_val is not None else input(f"{prompt}: ").strip()
            if not val_str and default_val is not None:
                return default_val
            return float(val_str)
        except ValueError:
            print("Invalid input! Please enter a valid decimal coordinate.")

def print_all_waypoints(waypoints):
    print("\n" + "=" * 82)
    print("                      COMPLETE LIST OF ALL FLIGHT WAYPOINTS")
    print("=" * 82)
    print(f"{'WP #':^6} | {'Latitude':^12} | {'Longitude':^12} | {'Leg Dist':^10} | {'Bearing':^9} | {'Direction':^9} | {'Cumulative':^10}")
    print("-" * 82)

    cumulative_dist = 0.0

    for i, (lat, lon) in enumerate(waypoints):
        if i == 0:
            print(f" {i:03d}   | {lat:12.6f} | {lon:12.6f} | {'START':^10} | {'---':^9} | {'LAUNCH':^9} | {0.0:8.1f} m")
        else:
            prev_lat, prev_lon = waypoints[i - 1]
            leg_dist = haversine_distance(prev_lat, prev_lon, lat, lon)
            bearing = calculate_bearing(prev_lat, prev_lon, lat, lon)
            cardinal = bearing_to_cardinal(bearing)
            cumulative_dist += leg_dist
            print(f" {i:03d}   | {lat:12.6f} | {lon:12.6f} | {leg_dist:7.1f} m  | {bearing:5.1f} deg | {cardinal:^9} | {cumulative_dist:8.1f} m")

    print("-" * 82)
    print(f"Total Waypoints: {len(waypoints)} | Total Route Distance: {cumulative_dist:.2f} m ({cumulative_dist / 1000.0:.3f} km)")
    print("=" * 82 + "\n")

def main():
    print("==================================================")
    print("       DRONE WAYPOINT PLANNER & ROUTE FINDER       ")
    print("==================================================")
    print("Choose an option:")
    print("  [1] Use Standard Test Route (Addis Ababa City Center)")
    print("  [2] Enter Custom GPS Coordinates")
    choice = input("Enter choice (1 or 2, default 1): ").strip()

    if choice == "2":
        print("\nEnter START Location:")
        start_lat = get_coordinate("Start Latitude")
        start_lon = get_coordinate("Start Longitude")

        print("\nEnter DESTINATION Location:")
        dest_lat = get_coordinate("Destination Latitude")
        dest_lon = get_coordinate("Destination Longitude")
    else:
        # Default Addis Ababa city center coordinates
        start_lat, start_lon = 9.03362, 38.73226
        dest_lat, dest_lon = 9.03262, 38.76343
        print(f"\nUsing default route: ({start_lat}, {start_lon}) -> ({dest_lat}, {dest_lon})")

    start_gps = (start_lat, start_lon)
    dest_gps = (dest_lat, dest_lon)

    print("\n[1/4] Fetching road network from OpenStreetMap...")
    padding = 0.01
    north = max(start_lat, dest_lat) + padding
    south = min(start_lat, dest_lat) - padding
    east = max(start_lon, dest_lon) + padding
    west = min(start_lon, dest_lon) - padding

    try:
        G = ox.graph.graph_from_bbox(
            bbox=(west, south, east, north),
            network_type="drive"
        )
    except Exception as e:
        print(f"Error fetching map data: {e}")
        return

    print("[2/4] Finding nearest road nodes...")
    start_node = ox.distance.nearest_nodes(G, X=start_lon, Y=start_lat)
    dest_node = ox.distance.nearest_nodes(G, X=dest_lon, Y=dest_lat)

    print("[3/4] Calculating shortest path along roads...")
    route = nx.shortest_path(G, start_node, dest_node, weight="length")

    gps_coordinates = []
    for node in route:
        lat = G.nodes[node]["y"]
        lon = G.nodes[node]["x"]
        gps_coordinates.append((lat, lon))

    print("[4/4] Generating HTML map...")
    route_map = folium.Map(location=start_gps, zoom_start=15)
    folium.PolyLine(gps_coordinates, weight=5, color="blue", tooltip="Road Path").add_to(route_map)
    folium.Marker(start_gps, popup="START", icon=folium.Icon(color="green")).add_to(route_map)
    folium.Marker(dest_gps, popup="DESTINATION", icon=folium.Icon(color="red")).add_to(route_map)
    route_map.save("drone_route.html")
    print("Interactive map saved to drone_route.html")

    # Display ALL waypoints with no truncation
    print_all_waypoints(gps_coordinates)

if __name__ == "__main__":
    main()