import os
import sys
# Add current directory to path to import ccd_preprocess
sys.path.append('.')

try:
    from ccd_preprocess import BLUE_211_METADATA, VIOLET_211_METADATA
except ImportError:
    print("Error: Could not import metadata from ccd_preprocess.py")
    sys.exit(1)

PROJECTS_DIR = "projects"

def verify_project(project_name, metadata):
    print(f"\n{'='*80}")
    print(f"VERIFYING PROJECT: {project_name}")
    print(f"{'='*80}")
    
    proj_path = os.path.join(PROJECTS_DIR, project_name)
    if not os.path.exists(proj_path):
        print(f"Error: Project directory {proj_path} not found.")
        return

    # Get recording files sorted by name (which is chronological for 'rec_YYYYMMDD...')
    files = sorted([f for f in os.listdir(proj_path) if f.startswith("rec_") and f.endswith(".npz")])
    
    if len(files) != len(metadata):
        print(f"WARNING: Mismatch in counts! Files: {len(files)}, Metadata entries: {len(metadata)}")
    else:
        print(f"Count Match: {len(files)} files found, {len(metadata)} metadata entries.")

    print(f"\n{'-'*85}")
    print(f"{'Seq':<4} | {'Filename':<30} | {'Exp. Conc (mg/L)':<18} | {'Exp. Time (ms)':<15}")
    print(f"{'-'*85}")

    # Limit display if mismatch length, otherwise show all
    limit = min(len(files), len(metadata))
    
    for i in range(limit):
        fname = files[i]
        conc, int_time = metadata[i]
        print(f"{i+1:<4} | {fname:<30} | {conc:<18.3f} | {int_time:<15}")

    if len(files) > limit:
        print(f"... and {len(files)-limit} more files without metadata.")
    if len(metadata) > limit:
        print(f"... and {len(metadata)-limit} more metadata entries without files.")

if __name__ == "__main__":
    verify_project("blue-211", BLUE_211_METADATA)
    verify_project("violet-211", VIOLET_211_METADATA)
