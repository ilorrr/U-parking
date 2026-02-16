from django.contrib import admin

# Register your models here.
from django.contrib import admin
from .models import Telemetry, HandicapSpot, PaidParking, ParkingSpot, VehicleType

@admin.register(Telemetry)
class TelemetryAdmin(admin.ModelAdmin):
    list_display = ("drone_id", "created_at")
    search_fields = ("drone_id",)
    ordering = ("-created_at",)

@admin.register(VehicleType)
class VehicleTypeAdmin(admin.ModelAdmin):
    list_display = ("id", "code", "display_name")
    search_fields = ("code", "display_name")
    ordering = ("display_name",)


@admin.register(ParkingSpot)
class ParkingSpotAdmin(admin.ModelAdmin):
    list_display = ("id", "lot_code", "label", "vehicle_type")
    search_fields = ("lot_code", "label", "vehicle_type__code", "vehicle_type__display_name")
    list_filter = ("lot_code", "vehicle_type")
    autocomplete_fields = ("vehicle_type",)


@admin.register(HandicapSpot)
class HandicapSpotAdmin(admin.ModelAdmin):
    list_display = ("id", "spot", "requires_permit", "signed")
    list_filter = ("requires_permit", "signed")
    search_fields = ("spot__lot_code", "spot__label")
    autocomplete_fields = ("spot",)


@admin.register(PaidParking)
class PaidParkingAdmin(admin.ModelAdmin):
    list_display = ("id", "spot", "hourly_rate", "currency", "is_paid")
    list_filter = ("is_paid", "currency")
    search_fields = ("spot__lot_code", "spot__label", "currency")
    autocomplete_fields = ("spot",)