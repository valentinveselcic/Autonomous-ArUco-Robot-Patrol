#!/usr/bin/env python3
import argparse
import pathlib

import cv2
from cv2 import aruco
import numpy as np

SDF_TEMPLATE = """<?xml version="1.0" ?>
<sdf version="1.7">
  <model name="aruco_marker_{id}">
    <static>true</static>
    <link name="link">
      <pose>0 0 -0.0005 0 0 1.5708</pose>
      <collision name="collision">
        <geometry>
          <box>
            <size>{size} {size} 0.001 </size>
          </box>
        </geometry>
      </collision>
      <visual name="visual">
        <geometry>
          <box>
            <size>{size} {size} 0.001 </size>
          </box>
        </geometry>
        <material>
          <ambient>1 1 1 1</ambient>
          <diffuse>1 1 1 1</diffuse>
          <specular>0 0 0 1</specular>
          <emissive>0 0 0 1</emissive>
          <pbr>
            <metal>
              <albedo_map>materials/textures/aruco_{id}.png</albedo_map>
              <roughness>0.4</roughness>
              <metalness>0.0</metalness>
            </metal>
          </pbr>
        </material>
      </visual>
    </link>
  </model>
</sdf>
"""


MODEL_CONFIG_TEMPLATE = """<?xml version="1.0"?>
<model>
  <name>aruco_marker_{id}</name>
  <version>1.0</version>
  <sdf version="1.7">model.sdf</sdf>
  <author>
    <name>generated</name>
    <email>n/a</email>
  </author>
  <description>ArUco marker {id}</description>
</model>
"""


def generate_marker_image(marker_id: int, side_px: int) -> np.ndarray:
    dictionary = aruco.Dictionary_get(aruco.DICT_4X4_50)
    img = np.zeros((side_px, side_px), dtype=np.uint8)
    aruco.drawMarker(dictionary, marker_id, side_px, img, borderBits=1)

    img_color = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return img_color


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dirpath",
        type=str,
        help="Path to root directory",
        default='src/aruco_patroller_task/models',

    )
    parser.add_argument(
        "--ids",
        type=int,
        nargs="+",
        default=[23, 42, 2],
        help="Marker IDs to generate",
    )
    parser.add_argument(
        "--size",
        type=float,
        default=0.3,
        help="Marker physical size in meters (edge length)",
    )
    parser.add_argument(
        "--pixels",
        type=int,
        default=400,
        help="Marker image size in pixels (edge length)",
    )
    args = parser.parse_args()

    pkg_path = pathlib.Path(args.dirpath).parent
    models_dir = pkg_path / "models"
    models_dir.mkdir(exist_ok=True)

    for m_id in args.ids:
        model_dir = models_dir / f"aruco_marker_{m_id}"
        textures_dir = model_dir / "materials" / "textures"
        textures_dir.mkdir(parents=True, exist_ok=True)

        img = generate_marker_image(m_id, args.pixels)
        png_path = textures_dir / f"aruco_{m_id}.png"
        cv2.imwrite(str(png_path), img)

        sdf_text = SDF_TEMPLATE.format(id=m_id, size=args.size)
        (model_dir / "model.sdf").write_text(sdf_text)

        cfg_text = MODEL_CONFIG_TEMPLATE.format(id=m_id)
        (model_dir / "model.config").write_text(cfg_text)

        print(f"Generated model for marker {m_id} at {model_dir}")


if __name__ == "__main__":
    main()