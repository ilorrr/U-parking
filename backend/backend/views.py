from django.http import response
from django.http import JsonResponse



def api_view(request):
    return JsonResponse({"API test"}, safe=True)
