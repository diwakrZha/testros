# src/fg_srv/rest_api/integrations/opcenter_integration.py

import os
import requests

def send_mission_to_opcenter(mission_data: dict, config: dict):
    """
    Example function to send mission data to Siemens Opcenter MES.
    """
    base_url = config.get("OPENCENTER_URL", "")
    username = config.get("OPENCENTER_USER", "")    
    password = config.get("OPENCENTER_PASS", "")

    # Example: Basic Authentication. Real systems might use token-based auth.
    auth = (username, password)

    try:
        response = requests.post(
            f"{base_url}/missions",
            json=mission_data,
            auth=auth,
            timeout=10
        )
        response.raise_for_status()
        return response.json()
    except requests.RequestException as e:
        return {"error": str(e), "system": "Opcenter"}
