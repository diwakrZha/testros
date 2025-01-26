# src/fg_srv/rest_api/app.py
# Author: Diwaker Jha
# Enhanced solution for session-based login, ERP config, graceful design

import os
import json
import threading
import time
import socket
import sys

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from fastapi import FastAPI, Request, Depends, Form
from fastapi.responses import HTMLResponse, JSONResponse, RedirectResponse
from fastapi.templating import Jinja2Templates
from fastapi.middleware.sessions import SessionMiddleware
from pydantic import BaseModel

import uvicorn
import paho.mqtt.client as mqtt

from .integrations.integrations_manager import broadcast_mission_to_integrations

# =========== ENV SETTINGS ============
MQTT_BROKER = os.getenv('MQTT_BROKER_HOST', 'mqtt_broker')
MQTT_PORT = int(os.getenv('MQTT_BROKER_PORT', 1883))
MQTT_TOPIC = 'missions'
MQTT_USERNAME = os.getenv('MQTT_USERNAME')
MQTT_PASSWORD = os.getenv('MQTT_PASSWORD')

FLEETGLUE_USER = os.getenv("FLEETGLUE_USERNAME", "admin")
FLEETGLUE_PASS = os.getenv("FLEETGLUE_PASSWORD", "admin")

# In-memory config loaded from environment variables
ERP_CONFIG = {
    "OPENCENTER_ENABLED": os.getenv("OPENCENTER_ENABLED", "True"),
    "OPENCENTER_URL": os.getenv("OPENCENTER_URL", ""),
    "OPENCENTER_USER": os.getenv("OPENCENTER_USER", ""),
    "OPENCENTER_PASS": os.getenv("OPENCENTER_PASS", ""),
    "SAP_MES_ENABLED": os.getenv("SAP_MES_ENABLED", "False"),
    "SAP_MES_URL": os.getenv("SAP_MES_URL", ""),
    "SAP_MES_USER": os.getenv("SAP_MES_USER", ""),
    "SAP_MES_PASS": os.getenv("SAP_MES_PASS", ""),
    "ODOO_ENABLED": os.getenv("ODOO_ENABLED", "False"),
    "ODOO_URL": os.getenv("ODOO_URL", ""),
    "ODOO_DB": os.getenv("ODOO_DB", ""),
    "ODOO_USER": os.getenv("ODOO_USER", ""),
    "ODOO_PASS": os.getenv("ODOO_PASS", ""),
    "SAP_ERP_ENABLED": os.getenv("SAP_ERP_ENABLED", "False"),
    "SAP_ERP_URL": os.getenv("SAP_ERP_URL", ""),
    "SAP_ERP_USER": os.getenv("SAP_ERP_USER", ""),
    "SAP_ERP_PASS": os.getenv("SAP_ERP_PASS", ""),
    "D365_ENABLED": os.getenv("D365_ENABLED", "False"),
    "D365_URL": os.getenv("D365_URL", ""),
    "D365_USER": os.getenv("D365_USER", ""),
    "D365_PASS": os.getenv("D365_PASS", ""),
}

# Shared mission data
current_mission = {}
current_mission_lock = threading.Lock()

# ========== FASTAPI APP SETUP =========
app = FastAPI()
templates = Jinja2Templates(directory="src/fg_srv/templates")

# Session middleware for 1-day login
app.add_middleware(
    SessionMiddleware,
    secret_key="SUPER-SECURE-KEY-HERE",
    max_age=86400  # 1 day in seconds
)

def is_logged_in(request: Request):
    return request.session.get("logged_in", False)

# ========== MODELS ===============
class Coordinates(BaseModel):
    latitude: float
    longitude: float

class Mission(BaseModel):
    mission_id: str
    mission_type: str
    target_coordinates: Coordinates
    priority: int

# ========== ROS2 + MQTT NODE =========
class MissionAPI(Node):
    def __init__(self):
        super().__init__('mission_api_node')
        self.publisher_ = self.create_publisher(String, 'mission_topic', 10)
        self.mqtt_client = mqtt.Client()
        if MQTT_USERNAME and MQTT_PASSWORD:
            self.mqtt_client.username_pw_set(MQTT_USERNAME, MQTT_PASSWORD)
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.connect_mqtt()

    def connect_mqtt(self):
        max_retries = 5
        for attempt in range(1, max_retries + 1):
            try:
                print(f"[MissionAPI] Connecting to MQTT {MQTT_BROKER}:{MQTT_PORT} (Attempt {attempt})")
                self.mqtt_client.connect(MQTT_BROKER, MQTT_PORT)
                print(f"[MissionAPI] MQTT Broker connected.")
                self.mqtt_client.loop_start()
                break
            except Exception as e:
                print(f"[MissionAPI] Attempt {attempt} failed: {e}")
                time.sleep(3)
        else:
            print("[MissionAPI] Could not connect to MQTT broker after retries.")
            self.destroy_node()
            rclpy.shutdown()
            sys.exit(1)

    def on_connect(self, client, userdata, flags, rc):
        if rc == 0:
            print("[MissionAPI] MQTT client connected, subscribing to topic:", MQTT_TOPIC)
            client.subscribe(MQTT_TOPIC)
        else:
            print("[MissionAPI] MQTT connect error, rc =", rc)

    def on_message(self, client, userdata, msg):
        try:
            payload = msg.payload.decode()
            print("[MissionAPI] MQTT msg payload:", payload)
            data = json.loads(payload)
            with current_mission_lock:
                global current_mission
                current_mission = data
            ros_msg = String()
            ros_msg.data = json.dumps(current_mission)
            self.publisher_.publish(ros_msg)
            print("[MissionAPI] Published mission data to ROS topic")
        except Exception as e:
            print("[MissionAPI] on_message error:", e)

mission_api_node = None

# ========== ROUTES ===============

@app.get("/login", response_class=HTMLResponse)
def login_page(request: Request):
    if request.session.get("logged_in", False):
        return RedirectResponse("/", status_code=302)
    return templates.TemplateResponse("login.html", {"request": request})

@app.post("/login_form", response_class=HTMLResponse)
async def login_form(request: Request):
    """Process the login form submission."""
    formdata = await request.form()
    username = formdata.get("username")
    password = formdata.get("password")
    if username == FLEETGLUE_USER and password == FLEETGLUE_PASS:
        request.session["logged_in"] = True
        return RedirectResponse("/", status_code=302)
    else:
        return templates.TemplateResponse(
            "login.html",
            {"request": request, "error_message": "Invalid credentials"},
            status_code=401
        )


@app.get("/", response_class=HTMLResponse)
def main_page(request: Request):
    if not request.session.get("logged_in", False):
        return RedirectResponse("/login", status_code=302)
    return templates.TemplateResponse("index.html", {"request": request})

@app.get("/get_config")
def get_config(request: Request):
    """Return ERP config if logged in, else 403."""
    if not request.session.get("logged_in", False):
        return JSONResponse(status_code=403, content={"error": "Not logged in"})
    return ERP_CONFIG

@app.post("/update_config")
async def update_config(request: Request):
    """Update in-memory ERP config from posted JSON."""
    if not is_logged_in(request):
        return JSONResponse(status_code=403, content={"error": "Not logged in"})
    data = await request.json()
    for key, val in data.items():
        if isinstance(val, bool):
            ERP_CONFIG[key] = "True" if val else "False"
        else:
            ERP_CONFIG[key] = val
    return ERP_CONFIG

@app.post("/mission")
def post_mission(request: Request, mission: Mission):
    """Create or update mission, publish to ROS & MQTT, broadcast to integrations."""
    if not is_logged_in(request):
        return JSONResponse(status_code=403, content={"error": "Not logged in"})

    global current_mission
    with current_mission_lock:
        current_mission = mission.dict()

    try:
        payload = json.dumps(current_mission)
        msg = String()
        msg.data = payload
        mission_api_node.publisher_.publish(msg)
        # MQTT
        if mission_api_node.mqtt_client:
            mission_api_node.mqtt_client.publish(MQTT_TOPIC, payload)
        # broadcast to MES/ERP with updated ERP_CONFIG
        results = broadcast_mission_to_integrations(current_mission, ERP_CONFIG)
        return {"status": "Mission received", "mission_id": mission.mission_id, "integration_results": results}
    except Exception as e:
        return {"status": "error", "message": str(e)}

@app.get("/mission")
def get_current_mission(request: Request):
    """Retrieve current mission if logged in."""
    if not is_logged_in(request):
        return JSONResponse(status_code=403, content={"error": "Not logged in"})
    with current_mission_lock:
        return current_mission

def get_container_ip():
    try:
        hostname = socket.gethostname()
        return socket.gethostbyname(hostname)
    except:
        return None

def main():
    global mission_api_node
    rclpy.init()
    mission_api_node = MissionAPI()

    ip = get_container_ip()
    if ip:
        print("[App] Container IP:", ip)
    host_ip = os.getenv('HOST_IP', '')
    if host_ip:
        print("[App] Host IP:", host_ip)

    # Start uvicorn in a background thread
    t = threading.Thread(
        target=uvicorn.run,
        args=(app,),
        kwargs={"host": "0.0.0.0", "port": 8000},
        daemon=True
    )
    t.start()

    rclpy.spin(mission_api_node)
    mission_api_node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
