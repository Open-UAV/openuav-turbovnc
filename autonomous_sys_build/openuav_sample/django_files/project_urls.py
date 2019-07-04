from django.urls import include, path
from django.contrib import admin

urlpatterns = [
    path('query/', include('query.urls')),
    path('admin/', admin.site.urls),
]