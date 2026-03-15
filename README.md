# Orbital Stick Mission Planner

This project is an advanced iteration of the falling stick physics simulator, scaled up to represent an Earth-scale orbital delivery mission.

## Features
- **Spherical ECEF Physics Backend**: The physics engine runs purely in NumPy, detached from rendering code. It computes 6-DOF coordinate integration and spherical targeting, exporting telemetry over a high-speed WebSocket at 20Hz.
- **MapLibre GL JS Visualization**: The web visualizer is a modern browser frontend that connects via WebSockets. It uses `MapLibre` to render a global orthographic-style 3D map, tracking the trailing flightpath with a glowing line and the stick position with a marker.
- **Multi-Phase State Machine**: The rocket attempts a full sub-orbital hop:
  1. `ASCENT`: Fires thrusters and tilts to navigate to the target coordinates.
  2. `COAST`: Ascends ballistically through the atmosphere.
  3. `DEPLOY`: At apogee/target, dispenses a mock satellite payload.
  4. `DESCENT`: Falls back towards the landing zone. 
  5. `LANDING`: Performs a suicide burn, guiding the stick vertically onto the target spot.

## Running the Simulator
To execute the Web Application:
```bash
pip install fastapi uvicorn websockets numpy
python3 server.py
```

Then, open your web browser and navigate to:
[http://localhost:8000](http://localhost:8000)

Fill in the mission parameters on the left sidebar and click "IGNITION" to track your spacecraft across the globe!
