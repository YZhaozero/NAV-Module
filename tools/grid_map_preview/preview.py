#!/usr/bin/env python3
"""Offline preview: official grid_map_pcl elevation -> local traversability."""
import argparse
import json
from pathlib import Path
import time
import numpy as np
from nav_app.map_processing import load_pcd, save_grid_map
from nav_app.features.navigation_bundle import digest
from nav_app.features.terrain_grid import build_terrain_grid

parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument('cloud', type=Path)
parser.add_argument('output', type=Path, help='new YAML filename; existing output is rejected')
parser.add_argument('--extractor', type=Path, required=True)
parser.add_argument('--surface-resolution', type=float, default=.20)
parser.add_argument('--resolution', type=float, default=.05)
parser.add_argument('--ground-z', type=float)
parser.add_argument('--max-slope', type=float, default=20.)
parser.add_argument('--max-step', type=float, default=.10)
parser.add_argument('--max-roughness', type=float, default=.04)
parser.add_argument('--max-gap', type=float, default=.45)
args = parser.parse_args()
output = args.output.with_suffix('.yaml')
for suffix in ('.yaml', '.pgm', '.analysis.npz', '.analysis.json'):
    if output.with_suffix(suffix).exists():
        parser.error('output already exists: '+str(output.with_suffix(suffix)))
if not .03 <= args.surface_resolution <= 1. or not .01 <= args.resolution <= 1.:
    parser.error('resolution must be within grid .01..1 m, surface .03..1 m')
started = time.monotonic()
cloud = load_pcd(args.cloud)
if np.prod(np.ceil(np.ptp(cloud.points[:, :2], axis=0)/args.surface_resolution)+1) > 2_000_000:
    parser.error('surface exceeds 2 million cells; increase --surface-resolution')
grid, layers, metadata = build_terrain_grid(cloud.points, resolution=args.resolution,
    floor_z=args.ground_z, extractor=args.extractor, surface_resolution=args.surface_resolution,
    max_slope_degrees=args.max_slope, max_step=args.max_step,
    max_roughness=args.max_roughness, max_gap=args.max_gap)
grid.frame_id = cloud.frame_id
metadata.update(source=str(args.cloud.resolve()), source_sha256=digest(args.cloud),
                field_validated=False, source_points=len(cloud.points),
                elapsed_seconds=time.monotonic()-started)
save_grid_map(output, grid)
np.savez_compressed(output.with_suffix('.analysis.npz'), **layers)
output.with_suffix('.analysis.json').write_text(json.dumps(metadata, ensure_ascii=False, indent=2)+'\n')
print(json.dumps(metadata, ensure_ascii=False, indent=2))
