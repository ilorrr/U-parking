from rest_framework.decorators import api_view
from rest_framework.response import Response
from api.models import ParkingSpot, Telemetry
from django.utils import timezone
from rest_framework import status

@api_view(["POST"])
def occupancy_update_api(request):
    
    section = request.data.get("section")
    spots = request.data.get("spots")

    if not section or not isinstance(spots, dict):
        return Response(
            {"detail": "Expected {section: int, spots: {label: bool}}"},
            status=status.HTTP_400_BAD_REQUEST,
        )

    section = int(section)
    now = timezone.now()

    updated = 0

    # Update known spots
    for label, occ in spots.items():
        updated += ParkingSpot.objects.filter(
            section=section,
            label=label
        ).update(
            is_occupied=bool(occ),
            last_seen_at=now,
        )

    # Clear stale spots (10 second timeout)
    from datetime import timedelta
    stale_cutoff = now - timedelta(seconds=10)

    ParkingSpot.objects.filter(
        section=section,
        last_seen_at__lt=stale_cutoff
    ).update(is_occupied=False)

    return Response({"status": "ok", "updated": updated,"section": section})

@api_view(["GET"])
def spaces_api(request):
    section = request.query_params.get("section")  # optional filter: ?section=1
    qs = ParkingSpot.objects.all()

    if section:
        qs = qs.filter(section=int(section))

    qs = qs.order_by("section", "label")

    data = [
        {
            "id": spot.id,
            "lot_code": spot.lot_code,
            "label": spot.label,
            "vehicle_type": spot.vehicle_type.code,
            "is_occupied": spot.is_occupied,
            "space": "Occupied" if spot.is_occupied else "Vacant",
        }
        for spot in qs
    ]
    return Response(data)

@api_view(["GET"])
def metrics_api(request):
    lot = request.query_params.get("lot")

    qs = ParkingSpot.objects.all()
    if lot:
        qs = qs.filter(lot_code=lot)

    total = qs.count()
    occupied = qs.filter(is_occupied=True).count()
    free = total - occupied

    last_telemetry = Telemetry.objects.order_by("-created_at").values_list("created_at", flat=True).first()

    return Response({
        "total": total,
        "occupied": occupied,
        "free": free,
        "occupancy_percent": round((occupied / total) * 100, 2) if total else 0,
        "last_telemetry": last_telemetry,
    })