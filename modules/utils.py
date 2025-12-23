import os
import time

def build_unique_output_path(source_desc, video_path, base_dir, prefix):
    """Bangun path file output unik berdasarkan sumber video."""
    timestamp = time.strftime("%Y%m%d_%H%M%S")

    if str(source_desc).upper() == "FILE":
        base_name = os.path.splitext(os.path.basename(video_path))[0]
        name = f"{prefix}_{base_name}_{timestamp}"
    else:
        name = f"{prefix}_Webcam_{timestamp}"

    ext = ".mp4"
    candidate = os.path.join(base_dir, name + ext)
    if not os.path.exists(candidate):
        return candidate

    i = 1
    while True:
        candidate = os.path.join(base_dir, f"{name}_{i}{ext}")
        if not os.path.exists(candidate):
            return candidate
        i += 1
