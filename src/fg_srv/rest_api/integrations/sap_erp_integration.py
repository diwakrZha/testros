# src/fg_srv/rest_api/integrations/sap_erp_integration.py

import os
import requests

def send_mission_to_sap_erp(mission_data: dict, config: dict):
    """
    Example function to send mission data to SAP ERP (stub).
    """
    base_url = config.get("SAP_ERP_URL", "")
    username = config.get("SAP_ERP_USER", "")
    password = config.get("SAP_ERP_PASS", "")

    try:
        # Possibly complex SOAP or OData calls in real usage.
        response = requests.post(
            f"{base_url}/import_mission",
            json=mission_data,
            auth=(username, password),
            timeout=10
        )
        response.raise_for_status()
        return response.json()
    except requests.RequestException as e:
        return {"error": str(e), "system": "SAP ERP"}
