"""Export locally downloaded complete NVIDIA rigs with per-clip calibration."""
import argparse
import json
import subprocess
import sys
from pathlib import Path

from pc_avm_preview import ROOT, DEFAULT_SCENE

DEPENDENCIES = ROOT / 'output/pc_surround_review/deps'
sys.path.insert(0, str(DEPENDENCIES))
from build_nvidia_avm_calibration import build

GROUPS = ['01_daylight_intersection', '02_daylight_side_traffic',
          '03_daylight_following', '04_clear_open_road', '05_overcast_following',
          '11_side_vehicle_bsd', '22_double_yellow_parked_cars', '26_night_urban_intersection']


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--groups', nargs='+', default=GROUPS)
    parser.add_argument('--video-fps', type=int, default=30,
                        help='Export real source samples at this frame rate (1-30)')
    args = parser.parse_args()
    if not 1 <= args.video_fps <= 30:
        parser.error('--video-fps must be between 1 and 30')
    material = DEFAULT_SCENE.parents[2]
    output = ROOT / 'output/pc_surround_groups'
    output.mkdir(parents=True, exist_ok=True)
    summaries = {}
    for path in output.glob('*/review.json'):
        if not (path.parent / 'nvidia_seven_camera_avm.mp4').is_file():
            continue
        report = json.loads(path.read_text(encoding='utf-8'))
        summaries[path.parent.name] = {'group': path.parent.name,
            'video': str(path.parent / 'nvidia_seven_camera_avm.mp4'),
            'clip_id': report['clip_id'], 'coverage': report['coverage_fractions_before_visual_fill'],
            'max_timestamp_difference_ms': report['max_timestamp_difference_ms'],
            'duration_seconds': report['duration_seconds'], 'export_fps': report['export_fps'],
            'antialias_ground': report.get('antialias_ground', False)}
    for name in args.groups:
        group = material / name
        scene = DEFAULT_SCENE if name == '11_side_vehicle_bsd' else group / 'original_four_view'
        target = output / name
        calibration = target / 'seven.yml'
        print(f'BUILD GROUP {name}', flush=True)
        build(group, calibration, 23.0)
        subprocess.run([sys.executable, str(ROOT / 'tools/pc_surround_review.py'),
            '--group', str(group), '--scene', str(scene), '--calibration', str(calibration),
            '--output', str(target), '--near-view', '--seam-refinement',
            '--antialias-ground',
            '--video-fps', str(args.video_fps)], check=True)
        report = json.loads((target / 'review.json').read_text(encoding='utf-8'))
        summaries[name] = {'group': name, 'video': str(target / 'nvidia_seven_camera_avm.mp4'),
            'clip_id': report['clip_id'], 'coverage': report['coverage_fractions_before_visual_fill'],
            'max_timestamp_difference_ms': report['max_timestamp_difference_ms'],
            'duration_seconds': report['duration_seconds'], 'export_fps': report['export_fps'],
            'antialias_ground': report.get('antialias_ground', False)}
        (output / 'groups.json').write_text(json.dumps([summaries[k] for k in sorted(summaries)],indent=2),encoding='utf-8')
    print(f'ALL {len(summaries)} GROUPS EXPORTED', flush=True)


if __name__ == '__main__':
    main()
