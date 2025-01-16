import os
import pandas as pd

def extract_objects_to_csv(input_file, output_csv):
    """
    Extracts object data from a Webots .wbt file and saves it to a CSV file.

    Parameters:
        input_file (str): Path to the input .wbt file.
        output_csv (str): Path to the output CSV file.
    """
    try:
        # Read the content of the input file
        with open(input_file, 'r') as file:
            file_content = file.readlines()

        objects = []
        current_object = None
        current_type = None

        for line in file_content:
            line = line.strip()

            # Detect a new object by its type (e.g., PlasticBarrel {)
            if line.endswith("{") and not line.startswith("Solid {"):
                current_type = line.split(" ")[0]  # The type is the first word before "{"

            # Detect a new Solid block
            if line.startswith("Solid {"):
                current_object = {"x": None, "y": None, "z": None, "type": current_type or "Unnamed"}
                current_type = None

            # Extract translation coordinates
            if line.startswith("translation") and current_object is not None:
                coords = line.split()[1:]  # Split and skip the "translation" keyword
                if len(coords) >= 3:
                    current_object["x"] = float(coords[0])
                    current_object["y"] = float(coords[1])
                    current_object["z"] = float(coords[2])

            # Extract object name if present
            if "name" in line and current_object is not None:
                name_parts = line.split('"')
                if len(name_parts) > 1:
                    current_object["type"] = name_parts[1]  # Extract the value inside quotes

            # Finalize the object when the block ends
            if line == "}" and current_object is not None:
                if current_object["x"] is not None and current_object["y"] is not None:
                    objects.append((current_object["x"], current_object["y"], current_object["z"], current_object["type"]))
                current_object = None

            # Handle standalone objects with type and translation in the same block
            if current_type and line.startswith("translation"):
                coords = line.split()[1:]  # Split and skip the "translation" keyword
                if len(coords) >= 3:
                    x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                    objects.append((x, y, z, current_type))
                    current_type = None

        # Create a DataFrame for structured storage
        df_objects = pd.DataFrame(objects, columns=["x", "y", "z", "type"])

        # Save to CSV
        df_objects.to_csv(output_csv, index=False)
        print(f"Objects successfully extracted to: {output_csv}")

    except Exception as e:
        print(f"An error occurred: {e}")

if __name__ == "__main__":
    input_file = "/ulstu/repositories/webots_ros2_suv/worlds/robocross_gazelle.wbt"
    output_csv = "/ulstu/repositories/webots_ros2_suv/worlds/robocross_gazelle.csv"

    if os.path.isfile(input_file):
        extract_objects_to_csv(input_file, output_csv)
    else:
        print(f"Input file '{input_file}' does not exist.")
