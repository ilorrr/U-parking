# api/views.py
from rest_framework.response import Response
from rest_framework.decorators import api_view

@api_view(['GET'])
def api_view(request):
    data = [
        {"id": 1, "Space": "Occupied"},
        {"id": 2, "Space": "Occupied"},
        {"id": 3, "Space": "Vacant"}
    ]
    return Response(data)  # DRF handles JSON automatically