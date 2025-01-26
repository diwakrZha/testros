# src/fg_srv/rest_api/integrations/integrations_manager.py

import os
from .opcenter_integration import send_mission_to_opcenter
from .sap_mes_integration import send_mission_to_sap_mes
from .odoo_integration import send_mission_to_odoo
from .sap_erp_integration import send_mission_to_sap_erp
from .d365_integration import send_mission_to_d365



def broadcast_mission_to_integrations(mission_data: dict, config: dict) -> dict:
    results = {}

    if config.get("OPENCENTER_ENABLED", "False").lower() == "true":
        results["opcenter"] = send_mission_to_opcenter(mission_data, config)

    if config.get("SAP_MES_ENABLED", "False").lower() == "true":
        results["sap_mes"] = send_mission_to_sap_mes(mission_data, config)

    if config.get("ODOO_ENABLED", "False").lower() == "true":
        results["odoo"] = send_mission_to_odoo(mission_data, config)

    if config.get("SAP_ERP_ENABLED", "False").lower() == "true":
        results["sap_erp"] = send_mission_to_sap_erp(mission_data, config)

    if config.get("D365_ENABLED", "False").lower() == "true":
        results["d365"] = send_mission_to_d365(mission_data, config)

    return results