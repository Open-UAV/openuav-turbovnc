from django.urls import path

from . import views

urlpatterns = [
	path('measures', views.measures, name='measures'),
	path('numUavs', views.numUavs, name='numUavs'),
	path('debugStmts', views.debugStmts, name='debugStmts'),
    path('', views.index, name='index'),
]