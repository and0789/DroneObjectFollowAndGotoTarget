[1] SIMULATION MODE (Tanpa Drone)
bash# Test tracking saja
python3 main.py --no-drone --tracker csrt
python3 main.py --no-drone --tracker yolo --target-class person

[2] TRACKING ONLY (Connect tapi tidak kontrol)
bash# Lihat tracking tapi drone tidak bergerak
python3 main.py --tracking-only --tracker csrt

[3] MANUAL TAKEOVER (Drone sudah terbang)
bash# Basic - drone sudah terbang
python3 main.py --skip-takeoff --tracker csrt

# Dengan RC Override (untuk ALT_HOLD)
python3 main.py --skip-takeoff --use-rc --tracker csrt

# Full manual (skip semua)
python3 main.py --skip-takeoff --skip-land --tracker csrt

# Tanpa ganti mode
python3 main.py --skip-takeoff --skip-mode-change --tracker csrt

[4] GPS BERMASALAH (ALT_HOLD)
bash# Drone di ALT_HOLD, kontrol via RC Override
python3 main.py --skip-takeoff --skip-mode-change --use-rc --tracker csrt

# Force ALT_HOLD mode
python3 main.py --skip-takeoff --force-mode alt_hold --use-rc --tracker csrt

[5] TRACKING MODE
bash# FOLLOW - jaga jarak
python3 main.py --skip-takeoff --mode follow --tracker csrt

# GOTO - dekati target
python3 main.py --skip-takeoff --mode goto --tracker csrt

[6] DISABLE KONTROL TERTENTU
bash# Hanya yaw (tidak maju/mundur, tidak naik/turun)
python3 main.py --skip-takeoff --no-forward-control --no-altitude-control --tracker csrt

# Hanya forward (tidak putar)
python3 main.py --skip-takeoff --no-yaw-control --tracker csrt

# Tanpa altitude control
python3 main.py --skip-takeoff --no-altitude-control --tracker csrt

[7] TRACKER OPTIONS
bashpython3 main.py --skip-takeoff --tracker csrt
python3 main.py --skip-takeoff --tracker dasiamrpn
python3 main.py --skip-takeoff --tracker yolo --target-class person
python3 main.py --skip-takeoff --tracker hybrid --target-class person

[8] OUTPUT OPTIONS
bash# Tanpa rekam video
python3 main.py --skip-takeoff --no-record --tracker csrt

# Custom output directory
python3 main.py --skip-takeoff --output-dir recordings --uhun tracker csrt

🎯 SKENARIO LAPANGAN
KondisiCommandTest tracking (tanpa drone)--no-droneLihat tracking saja--tracking-onlyDrone sudah terbang--skip-takeoffGPS bermasalah--skip-mode-change --use-rcJangan land otomatis--skip-landMode dekati target--mode gotoTrack orang saja--tracker yolo --target-class personTanpa naik/turun--no-altitude-controlTanpa putar--no-yaw-control

⌨️ KEYBOARD CONTROLS
KeyFunctionSSelect target (ROI)AAuto-select nearest (YOLO)PAuto-select person (YOLO)RRelease target1Mode FOLLOW2Mode GOTOGSet GUIDED modeLSet LOITER modeHSet ALT_HOLD modeTToggle tracking-onlySPACEPause/ResumeQQuit

📖 Lihat Help Lengkap
python3 main.py --help
