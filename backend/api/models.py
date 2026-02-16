"""Canonical Django ORM schemas for parking inventory and pricing.

The dashboard backend should import these models directly so both backend
surfaces stay aligned.
"""

from django.core.exceptions import ValidationError
from django.db import models

class Telemetry(models.Model):
    created_at = models.DateTimeField(auto_now_add=True)
    drone_id = models.CharField(max_length=64)
    payload = models.JSONField()  # stores raw feed/telemetry

    def __str__(self):
        return f"{self.drone_id} @ {self.created_at}"


class VehicleType(models.Model):
    """Supported vehicle category (e.g., sedan, SUV, motorcycle)."""

    code = models.CharField(max_length=32, unique=True)
    display_name = models.CharField(max_length=64)

    class Meta:
        db_table = "vehicle_types"
        ordering = ["display_name"]

    def __str__(self) -> str:
        return self.display_name


class ParkingSpot(models.Model):
    """Core parking spot schema."""

    lot_code = models.CharField(max_length=32)
    label = models.CharField(max_length=32)
    vehicle_type = models.ForeignKey(
        VehicleType,
        on_delete=models.PROTECT,
        related_name="parking_spots",
    )

    class Meta:
        db_table = "parking_spots"
        constraints = [
            models.UniqueConstraint(
                fields=["lot_code", "label"],
                name="unique_spot_per_lot",
            )
        ]

    def __str__(self) -> str:
        return f"{self.lot_code}-{self.label}"


class HandicapSpot(models.Model):
    """Schema for ADA/accessible parking spots."""

    spot = models.OneToOneField(
        ParkingSpot,
        on_delete=models.CASCADE,
        related_name="handicap_details",
    )
    requires_permit = models.BooleanField(default=True)
    signed = models.BooleanField(default=True)

    class Meta:
        db_table = "handicap_spots"

    def __str__(self) -> str:
        return f"Handicap: {self.spot}"


class PaidParking(models.Model):
    """Schema for paid parking configuration."""

    spot = models.OneToOneField(
        ParkingSpot,
        on_delete=models.CASCADE,
        related_name="paid_parking",
    )
    hourly_rate = models.DecimalField(max_digits=8, decimal_places=2)
    currency = models.CharField(max_length=3, default="USD")
    is_paid = models.BooleanField(default=True)

    class Meta:
        db_table = "paid_parking"

    def clean(self) -> None:
        super().clean()
        if self.hourly_rate < 0:
            raise ValidationError({"hourly_rate": "Hourly rate cannot be negative."})
        self.currency = self.currency.upper()

    def __str__(self) -> str:
        return f"PaidParking({self.spot}, {self.hourly_rate} {self.currency})"