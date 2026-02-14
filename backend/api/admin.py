from django.contrib import admin

# Register your models here.
from django.contrib import admin
from .models import Telemetry

@admin.register(Telemetry)
class TelemetryAdmin(admin.ModelAdmin):
    list_display = ("drone_id", "created_at")
    search_fields = ("drone_id",)
    ordering = ("-created_at",)