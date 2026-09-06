import os
from route_planner import generate_route

SKETCH_FOLDER = r"C:\Users\jabes\Desktop\DRONE AUTOMATION\Final_Drone_Automation"
HEADER_PATH = os.path.join(SKETCH_FOLDER, "waypoints.h")

def export_waypoints_header(waypoints, filepath=HEADER_PATH):
    """Generates C++ array header file for Arduino static compilation."""
    total = len(waypoints)
    with open(filepath, "w") as f:
        f.write("/* AUTO-GENERATED WAYPOINTS FILE - DO NOT EDIT MANUALLY */\n")
        f.write("#ifndef WAYPOINTS_H\n")
        f.write("#define WAYPOINTS_H\n\n")
        f.write("struct Waypoint {\n")
        f.write("    double lat;\n")
        f.write("    double lon;\n")
        f.write("};\n\n")
        f.write(f"const int TOTAL_WAYPOINTS = {total};\n\n")
        f.write("const Waypoint pathHeader[] = {\n")
        for idx, (lat, lon) in enumerate(waypoints):
            f.write(f"    {{{lat:.6f}, {lon:.6f}}}")
            if idx < total - 1:
                f.write(",\n")
            else:
                f.write("\n")
        f.write("};\n\n")
        f.write("#endif // WAYPOINTS_H\n")
    print(f"\n[SUCCESS] Saved {total} waypoints directly to: '{filepath}'")

def print_all_waypoints(waypoints):
    print("\n" + "=" * 60)
    print("           COMPLETE LIST OF ALL FLIGHT WAYPOINTS")
    print("=" * 60)
    print(f"{'WP #':^6} | {'Latitude':^18} | {'Longitude':^18}")
    print("-" * 60)
    for i, (lat, lon) in enumerate(waypoints):
        print(f" {i:03d}   | {lat:18.6f} | {lon:18.6f}")
    print("-" * 60)
    print(f"Total Waypoints: {len(waypoints)}")
    print("=" * 60 + "\n")


def get_coordinate_input(label):
    while True:
        try:
            return float(input(f"Enter {label}: "))
        except ValueError:
            print("Invalid input. Please enter a valid numerical coordinate.")


def main():
    print("==================================================================")
    print("      DRONE GROUND STATION: ROUTE GENERATOR & HEADER EXPORTER     ")
    print("==================================================================")

    print("\nSelect Mission Mode:")
    print("  [1] Standard Quick-Test Route (Sofia City Center)")
    print("  [2] Custom GPS Start & Destination Coordinates")
    mode = input("Enter choice (1 or 2, default 1): ").strip()

    if mode == "2":
        print("\n--- START LOCATION ---")
        start_lat = get_coordinate_input("Start Latitude")
        start_lon = get_coordinate_input("Start Longitude")

        print("\n--- DESTINATION LOCATION ---")
        dest_lat = get_coordinate_input("Destination Latitude")
        dest_lon = get_coordinate_input("Destination Longitude")
    else:
        start_lat, start_lon = 42.696665, 23.321870
        dest_lat, dest_lon = 42.695100, 23.328400
        print(f"\nUsing default Sofia route: ({start_lat}, {start_lon}) -> ({dest_lat}, {dest_lon})")

    start_gps = (start_lat, start_lon)
    end_gps = (dest_lat, dest_lon)

    # 1. Fetch Waypoints from route_planner.py
    waypoints = generate_route(start_gps, end_gps)
    if not waypoints:
        print("Failed to compute a valid route. Aborting mission.")
        return

    # 2. Display Waypoints
    print_all_waypoints(waypoints)

    # 3. Export directly to the Arduino sketch folder
    export_waypoints_header(waypoints, HEADER_PATH)

    print("\n------------------------------------------------------------------")
    print("NEXT STEPS FOR PROTEUS SIMULATION:")
    print(f"1. Verified 'waypoints.h' is placed inside '{SKETCH_FOLDER}'.")
    print("2. Open Arduino IDE and press Ctrl + Alt + S (Export Compiled Binary).")
    print("3. Load the output .hex file into the Arduino in Proteus and run!")
    print("------------------------------------------------------------------")


if __name__ == "__main__":
    main()