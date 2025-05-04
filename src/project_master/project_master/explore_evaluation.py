import numpy as np
from project_interfaces.msg import Object

def read_map(filename):
    try:
        objects = []
        with open(filename, "r") as file:
            for line in file:
                parts = line.strip().split(',')
                if len(parts) < 4:
                    print(f"Skipping invalid line: {line}")
                    continue
                object_msg = Object()
                if parts[0] == "1":
                    object_msg.object_type = Object.CUBE
                elif parts[0] == "2":
                    object_msg.object_type = Object.SPHERE
                elif parts[0] == "3":
                    object_msg.object_type = Object.PLUSHIE
                elif parts[0] == "B":
                    object_msg.object_type = Object.BOX
                else:
                    print(f"Skipping line due to unknown object type: {line}")
                    continue
                object_msg.x = float(parts[1])/100
                object_msg.y = float(parts[2])/100
                object_msg.angle = float(parts[3])
                objects.append(object_msg)
        return objects
    except FileNotFoundError:
        print(f"Map file ({filename}) not found!")
        return []

def compare_object_lists(correct_object_list, found_object_list):
    score = 0
    for found_object in found_object_list:
        match_found = False
        for correct_object in correct_object_list:
            if found_object.object_type != correct_object.object_type:
                continue
            distance = np.hypot(
                correct_object.x - found_object.x,
                correct_object.y - found_object.y,
            )
            if distance <= 0.20:
                match_found = True
                score += 1
                correct_object_list.remove(correct_object)
                break
        if not match_found:
            print(f"No match for (object type: {found_object.object_type}, x: {found_object.x:.2f}, y: {found_object.y:.2f})")
            score -= 1
    return score

if __name__ == "__main__":
    correct_map_file = "maps/correct_map.csv"
    found_map_file = "maps/map.csv"

    correct_object_list = read_map(correct_map_file)
    found_object_list = read_map(found_map_file)
    score = compare_object_lists(correct_object_list, found_object_list)
    print(f"Exploration phase score is: {score}")
    
