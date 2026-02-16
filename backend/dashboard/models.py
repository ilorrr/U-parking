from django.contrib import admin
from django.core.exceptions import ValidationError
from django.db import models
"""Dashboard backend models.

Re-export backend API schemas so the dashboard and API always use the
same model definitions.
"""

from api.models import Telemetry, HandicapSpot, PaidParking, ParkingSpot, VehicleType

__all__ = [
    "Telemetry",
    "ParkingSpot",
    "VehicleType",
    "HandicapSpot",
    "PaidParking",
]
