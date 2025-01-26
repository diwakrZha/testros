# src/fg_srv/test/test_rest_api.py

import pytest
from fastapi.testclient import TestClient

from rest_api.app import app, current_mission_lock, current_mission

client = TestClient(app)

@pytest.fixture
def mission_data():
    return {
        "mission_id": "123",
        "mission_type": "Delivery",
        "target_coordinates": {"latitude": 12.9716, "longitude": 77.5946},
        "priority": 2
    }

def test_post_mission(mission_data):
    response = client.post("/mission", json=mission_data)
    assert response.status_code == 200
    data = response.json()
    assert data["status"] == "Mission received"
    assert data["mission_id"] == mission_data["mission_id"]

def test_get_mission(mission_data):
    # Post a mission
    client.post("/mission", json=mission_data)
    # Retrieve it
    response = client.get("/mission")
    assert response.status_code == 200
    data = response.json()
    assert data == mission_data
