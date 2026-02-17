from rest_framework.decorators import api_view
from rest_framework.response import Response
from api.models import ParkingSpot, Telemetry

@api_view(["GET"])
def spaces_api(request):
    spots = ParkingSpot.objects.all().order_by("lot_code", "label")

    data = [
        {
            "id": spot.id,
            "lot_code": spot.lot_code,
            "label": spot.label,
            "vehicle_type": spot.vehicle_type.code,
            "space": "Unknown",  # until you store occupancy
        }
        for spot in spots
    ]
    return Response(data)

@api_view(["GET"])
def metrics_api(request):
    lot = request.query_params.get("lot")

    qs = ParkingSpot.objects.all()
    if lot:
        qs = qs.filter(lot_code=lot)

    total = qs.count()

    # You don't have occupancy stored yet, so we can't compute occupied/free accurately.
    # We'll return 0 occupied, total free, until telemetry->occupancy logic is added.
    occupied = 0
    free = total

    last_telemetry = Telemetry.objects.order_by("-created_at").values_list("created_at", flat=True).first()

    return Response({
        "total": total,
        "occupied": occupied,
        "free": free,
        "occupancy_percent": round((occupied / total) * 100, 2) if total else 0,
        "last_telemetry": last_telemetry,
    })