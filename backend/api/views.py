from rest_framework.decorators import api_view
from rest_framework.response import Response

@api_view(['GET'])
def spaces_api(request):
    data = [
        {"id": 1, "space": "Occupied"},
        {"id": 2, "space": "Occupied"},
        {"id": 3, "space": "Vacant"},
    ]
    return Response(data)