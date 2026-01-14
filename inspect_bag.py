"""
Check types of topics in a bag file
"""
from rosbags.rosbag1 import Reader
from rosbags.typesys import get_typestore, Stores
import sys
from pathlib import Path

def inspect_bag(bag_path):
    print(f"Inspecting bag: {bag_path}")
    with Reader(bag_path) as reader:
        for conn in reader.connections:
            print(f"Topic: {conn.topic}")
            print(f"  Type: {conn.msgtype}")
            print(f"  MD5: {conn.digest}")
            print("-" * 20)

if __name__ == "__main__":
    base_dir = Path(r"c:\Users\Dendrobium\Desktop\Personal Docs\HKUST\LunarSimulator\dataset")
    subdirs = [d for d in base_dir.iterdir() if d.is_dir()]
    if not subdirs:
        print("No dataset subdir found")
        sys.exit(1)
    
    latest_dir = max(subdirs, key=lambda x: x.stat().st_mtime)
    
    # Try dataset_final.bag (the one user is using)
    bag_file = latest_dir / "dataset_final.bag"
    
    if not bag_file.exists():
        print("dataset_final.bag not found, checking dataset.bag")
        bag_file = latest_dir / "dataset.bag"
    
    if not bag_file.exists():
        print("No bag found")
        sys.exit(1)
        
    inspect_bag(str(bag_file))
