from rest_framework.decorators import api_view
from rest_framework.response import Response
from dashboard.models import ParkingSpot

@api_view(['GET'])
def spaces_api(request):
    spots = ParkingSpot.objects.all().order_by("spot_id")

    data = [
        {
            "id": spot.spot_id,
            "space": "Occupied" if spot.is_occupied else "Vacant"
        }
        for spot in spots
    ]

    return Response(data)
