# src/fg_srv/rest_api/integrations/d365_integration.py

import os
import requests

def send_mission_to_d365(mission_data: dict, config: dict):
    """
    Example function to send mission data to Microsoft Dynamics 365 ERP.
    """
    base_url = config.get("D365_URL", "")
    username = config.get("D365_USER", "")
    password = config.get("D365_PASS", "")

    try:
        response = requests.post(
            f"{base_url}/missions",
            json=mission_data,
            auth=(username, password),
            timeout=10
        )
        response.raise_for_status()
        return response.json()
    except requests.RequestException as e:
        return {"error": str(e), "system": "D365"}
