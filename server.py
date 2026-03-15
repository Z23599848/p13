import uvicorn
from fastapi import FastAPI, WebSocket, WebSocketDisconnect
from fastapi.staticfiles import StaticFiles
from fastapi.responses import HTMLResponse
import asyncio
import json
import os

from physics import RocketSimulation

app = FastAPI()

# Mount static files (HTML, JS, CSS)
os.makedirs("web", exist_ok=True)
app.mount("/static", StaticFiles(directory="web"), name="static")

@app.get("/")
async def get():
    with open("web/index.html", "r") as f:
        return HTMLResponse(f.read())

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    
    sim = None
    
    try:
        while True:
            # Wait for config from client to start
            data = await websocket.receive_text()
            config = json.loads(data)
            
            print(f"Received mission config: {config}")
            sim = RocketSimulation(config)
            
            # Run simulation loop
            # The python backend runs dt=0.05 (20hz). We can warp time easily here.
            TIME_WARP = 2 
            
            while sim.current_phase != 5: # 5 = LANDED
                try:
                    # Compute multiple steps per tick to achieve time warp
                    for _ in range(TIME_WARP):
                        telemetry = sim.step()
                    
                    # Send latest state to browser
                    await websocket.send_json(telemetry)
                except Exception as e:
                    print(f"Error during simulation step: {e}")
                    await websocket.send_json({"error": str(e)})
                    break
                
                # Sleep to match real time (20Hz broadcast rate)
                await asyncio.sleep(0.05)
                
            # Send final landed state
            await websocket.send_json(sim._get_telemetry())
            print("Mission completed.")
            
    except WebSocketDisconnect:
        print("Client disconnected")

if __name__ == "__main__":
    print("Starting server on http://localhost:8000")
    uvicorn.run(app, host="0.0.0.0", port=8000)
