# src/fg_srv/rest_api/integrations/sap_mes_integration.py

import os
import requests

def send_mission_to_sap_mes(mission_data: dict, config: dict):
    """
    Example function to send mission data to SAP MES.
    """
    base_url = config.get("SAP_MES_URL", "")
    username = config.get("SAP_MES_USER", "")
    password = config.get("SAP_MES_PASS", "")

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
        return {"error": str(e), "system": "SAP MES"}
