# Project Structure

```
room-activity-scan/
├── embedded_programming/           # Embedded firmware projects
│   ├── activity_monitor/           # Main room activity monitoring firmware
│   │   ├── main/
│   │   │   ├── main.c              # Main application code (HLK-LD2420 sensor)
│   │   │   └── CMakeLists.txt
│   │   └── CMakeLists.txt          # Project configuration
│   └── new_hlkld2420/              # Original test program (legacy)
├── docs/                           # Documentation
├── images/                         # Project images and diagrams
├── README.md                       # Project overview and setup guide
├── LICENSE                         # MIT License
└── .gitignore                      # Git ignore rules
```

## Working with the Project

### Navigate to the firmware project:
```bash
cd embedded_programming/activity_monitor
```

### Build and flash:
```bash
idf.py set-target esp32s3
idf.py build
idf.py -p /dev/ttyUSB0 flash monitor
```

## Adding New Firmware Projects

Create new ESP-IDF projects under `embedded_programming/` directory:

```bash
cd embedded_programming
idf.py create-project <your-project-name>
```
