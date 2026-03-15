// Initialize Cesium Viewer
const viewer = new Cesium.Viewer('cesiumContainer', {
    baseLayerPicker: true,
    geocoder: false,
    homeButton: true,
    infoBox: false,
    navigationHelpButton: false,
    sceneModePicker: true,
    timeline: false,
    animation: false,
    shouldAnimate: true,
});

// Set a reliable imagery provider
viewer.imageryLayers.addImageryProvider(new Cesium.OpenStreetMapImageryProvider({
    url: 'https://tile.openstreetmap.org/'
}));

viewer.scene.globe.enableLighting = true;
viewer.scene.globe.show = true;
viewer.scene.skyAtmosphere.show = true;

// Custom Rocket Entity (SpaceX Falcon 9 Dimensions: 70m x 3.7m)
const rocketEntity = viewer.entities.add({
    name: "Falcon 9 Booster",
    position: Cesium.Cartesian3.fromDegrees(0, 0, 0),
    cylinder: {
        length: 70.0,
        topRadius: 1.85,
        bottomRadius: 1.85,
        material: Cesium.Color.WHITE,
        outline: true,
        outlineColor: Cesium.Color.BLACK
    },
    label: {
        text: 'FALCON 9',
        font: '14pt monospace',
        style: Cesium.LabelStyle.FILL_AND_OUTLINE,
        outlineWidth: 2,
        verticalOrigin: Cesium.VerticalOrigin.BOTTOM,
        pixelOffset: new Cesium.Cartesian2(0, -50),
        disableDepthTestDistance: Number.POSITIVE_INFINITY
    }
});

// Trajectory Line
let trajectoryPositions = [];
const trajectoryEntity = viewer.entities.add({
    name: 'Trajectory',
    polyline: {
        positions: new Cesium.CallbackProperty(() => trajectoryPositions, false),
        width: 4,
        material: new Cesium.PolylineGlowMaterialProperty({
            glowPower: 0.2,
            color: Cesium.Color.CYAN
        })
    }
});

let ws = null;
let isTracking = false;

document.getElementById('start-btn').addEventListener('click', () => {
    // Get config
    const config = {
        launch_lat: parseFloat(document.getElementById('l_lat').value),
        launch_lon: parseFloat(document.getElementById('l_lon').value),
        payload_alt: parseFloat(document.getElementById('p_alt').value),
        payload_lat: parseFloat(document.getElementById('p_lat').value),
        payload_lon: parseFloat(document.getElementById('p_lon').value),
        land_lat: parseFloat(document.getElementById('ld_lat').value),
        land_lon: parseFloat(document.getElementById('ld_lon').value),
    };

    trajectoryPositions = []; // Reset trajectory

    if (ws) ws.close();
    ws = new WebSocket(`ws://${location.host}/ws`);

    ws.onopen = () => {
        document.getElementById('start-btn').textContent = "IN FLIGHT...";
        document.getElementById('start-btn').classList.replace('bg-blue-600', 'bg-red-600');
        ws.send(JSON.stringify(config));

        // Setup camera tracking
        viewer.trackedEntity = rocketEntity;
        isTracking = true;
    };

    ws.onmessage = (event) => {
        const data = JSON.parse(event.data);

        // Update dashboard
        document.getElementById('t-phase').textContent = data.phase;
        document.getElementById('t-time').textContent = data.time.toFixed(1) + " s";
        document.getElementById('t-alt').textContent = Math.round(data.alt) + " m";
        document.getElementById('t-vel').textContent = Math.round(data.velocity) + " m/s";

        // Update Cesium Entities
        const position = Cesium.Cartesian3.fromDegrees(data.lon, data.lat, data.alt);
        rocketEntity.position = position;

        // Apply quaternion [x, y, z, w] -> Cesium uses [x,y,z,w] or creates via new Cesium.Quaternion(x,y,z,w)
        const q = data.quaternion;
        rocketEntity.orientation = new Cesium.Quaternion(q[0], q[1], q[2], q[3]);

        // Push trajectory
        trajectoryPositions.push(position);

        if (data.phase === "LANDED") {
            document.getElementById('start-btn').textContent = "MISSION COMPLETE (RESTART)";
            document.getElementById('start-btn').classList.replace('bg-red-600', 'bg-green-600');
            ws.close();
        }
    };

    ws.onclose = () => {
        if (document.getElementById('t-phase').textContent !== "LANDED") {
            document.getElementById('start-btn').textContent = "CONNECTION LOST";
            document.getElementById('start-btn').classList.replace('bg-red-600', 'bg-red-800');
        }
    }
});
