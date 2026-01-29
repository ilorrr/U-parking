from django.http import JsonResponse

# Existing dashboard index view
def index(request):
    return JsonResponse({"message": "Dashboard index"})  # optional

# New health endpoint
def health(request):
    return JsonResponse({"status": "ok"})
