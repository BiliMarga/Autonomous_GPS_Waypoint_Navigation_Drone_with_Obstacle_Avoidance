import shutup
shutup.please()

import networkx as nx
import osmnx as ox
import folium

def generate_route(start_gps, end_gps, max_waypoints=25):
    start_lat, start_lon = start_gps
    end_lat, end_lon = end_gps

    print("\nCalculating route...")
    padding = 0.01
    north = max(start_lat, end_lat) + padding
    south = min(start_lat, end_lat) - padding
    east = max(start_lon, end_lon) + padding
    west = min(start_lon, end_lon) - padding

    try:
        G = ox.graph.graph_from_bbox(
            bbox=(west, south, east, north),
            network_type="drive"
        )
    except Exception as e:
        print(f"Error downloading network map: {e}")
        return []

    start_node = ox.distance.nearest_nodes(G, X=start_lon, Y=start_lat)
    end_node = ox.distance.nearest_nodes(G, X=end_lon, Y=end_lat)

    route = nx.shortest_path(G, start_node, end_node, weight="length")

    # Collect full node coordinates along the path
    full_coordinates = []
    for node in route:
        lat = G.nodes[node]["y"]
        lon = G.nodes[node]["x"]
        full_coordinates.append((lat, lon))

    # Calculate distance metrics on the full path
    route_edges = ox.routing.route_to_gdf(G, route, weight="length")
    total_distance = route_edges["length"].sum()

    # Downsample path to enforce waypoint limit strictly under 50
    if len(full_coordinates) > max_waypoints:
        step = (len(full_coordinates) - 1) / (max_waypoints - 1)
        gps_coordinates = [
            full_coordinates[int(round(i * step))] 
            for i in range(max_waypoints)
        ]
        # Guarantee exact destination matching
        gps_coordinates[-1] = full_coordinates[-1]
    else:
        gps_coordinates = full_coordinates

    print(f"Raw node count: {len(full_coordinates)}.")
    print(f"Route exported with {len(gps_coordinates)} waypoints (Max limit: {max_waypoints}).")
    print(f"Total Distance: {total_distance:.2f} meters ({total_distance / 1000:.3f} km)")

    # Generate HTML Map
    route_map = folium.Map(location=start_gps, zoom_start=15)
    folium.PolyLine(gps_coordinates, weight=5, color="blue", tooltip="Flight Path").add_to(route_map)
    folium.Marker(start_gps, popup="START", icon=folium.Icon(color="green")).add_to(route_map)
    folium.Marker(end_gps, popup="DESTINATION", icon=folium.Icon(color="red")).add_to(route_map)
    route_map.save("drone_route.html")
    print("Map generated: drone_route.html")

    return gps_coordinates