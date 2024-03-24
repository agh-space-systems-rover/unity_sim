import os
import cv2
from cv2 import aruco

dicts_to_generate = {
    aruco.DICT_4X4_50: ("4X4_50", 50, 4),
}
white_border = 1

script_dir = os.path.dirname(__file__)
textures_dir = f"{script_dir}/Assets/Simulation/ArUco/Textures"

# Generate all possible markers and save them to ./Textures/{dict_name}/{id}.png
for dict_id, (dict_name, dict_size, marker_size) in dicts_to_generate.items():
    aruco_dict = aruco.getPredefinedDictionary(dict_id)
    out_dir = f"{textures_dir}/{dict_name}"
    os.makedirs(out_dir, exist_ok=True)
    for i in range(dict_size):
        marker = aruco.generateImageMarker(aruco_dict, i, marker_size + 2)

        # Add white border
        marker = cv2.copyMakeBorder(
            marker,
            white_border,
            white_border,
            white_border,
            white_border,
            cv2.BORDER_CONSTANT,
            value=[255, 255, 255],
        )

        cv2.imwrite(f"{out_dir}/{i}.png", marker)
        print(f"Generated marker {i} from dictionary {dict_name}")
