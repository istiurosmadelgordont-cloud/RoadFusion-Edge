"""Build scene/avm.yml from measured correspondences; never invent ground points.
Input JSON: {"front": {"image_points": [[u,v],...], "ground_points": [[x,y],...]},...}
Image coordinates normalized [0,1]; ground metres: x right, y forward, ego centre origin.
Four corners must follow the same perimeter order in both arrays.
"""
import argparse
import json
from pathlib import Path
import cv2
import numpy as np

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('measurements', type=Path)
parser.add_argument('output', type=Path)
args = parser.parse_args()
data = json.loads(args.measurements.read_text(encoding='utf-8'))
validated = {}
for name in ('front', 'rear', 'left', 'right'):
    if name not in data:
        continue
    image = np.asarray(data[name]['image_points'], np.float32)
    ground = np.asarray(data[name]['ground_points'], np.float32)
    for points in (image, ground):
        if points.shape != (4, 2) or not np.isfinite(points).all() or not cv2.isContourConvex(points):
            raise ValueError(f'{name}: requires four finite convex perimeter points')
    if (image < 0).any() or (image > 1).any() or (abs(ground) > 20).any():
        raise ValueError(f'{name}: coordinates out of range')
    if abs(cv2.contourArea(image)) < .001 or abs(cv2.contourArea(ground)) < .01:
        raise ValueError(f'{name}: degenerate area')
    validated[name] = (image, ground)
if not validated:
    raise ValueError('No camera measurements')
# OpenCV vector<Point2f> serialization is a flat YAML sequence.
lines = ['%YAML:1.0', '---', 'version: 1']
for name, (image, ground) in validated.items():
    lines.append(name + ':')
    for key, value in [('image_points', image), ('ground_points', ground)]:
        lines.append('  ' + key + ': [ ' + ', '.join(f'{float(v):.8g}' for v in value.ravel()) + ' ]')
args.output.write_text('\n'.join(lines) + '\n', encoding='utf-8')
print(f'Saved {args.output}: {len(validated)}/4 cameras; uncalibrated areas remain blank')
