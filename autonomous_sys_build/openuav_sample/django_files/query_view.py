from django.http import HttpResponse
import subprocess

def index(request):
    return HttpResponse("Hello, world. You're at the query index.")

def numUavs(request):
	cmd = '''cat /simulation/inputs/parameters/swarm.sh | grep num_uavs | grep -o -E '([0-9]|[1-9][0-9])' | tail -n1 '''

	try:
		p1 = subprocess.Popen(['cat', '/simulation/inputs/parameters/swarm.sh'], stdout=subprocess.PIPE)
		p2 = subprocess.Popen(['grep', 'num_uavs'], 
			stdin=p1.stdout, stdout=subprocess.PIPE)
		p1.stdout.close()
		p3 = subprocess.Popen(['grep', '-o' , '-E', '''([0-9]|[1-9][0-9])'''],
			stdin=p2.stdout,stdout=subprocess.PIPE)
		p2.stdout.close()
		p4 = subprocess.Popen(['tail', '-n1'], 
			stdin=p3.stdout, stdout=subprocess.PIPE)
		p3.stdout.close()
		output = p4.communicate()[0]
	except:
		output = 'Nothing'

	output = output.decode('UTF-8').strip() + '#'
	return HttpResponse(output)

def measures(request):
	cmd = '''ps ax | grep measure | wc -l'''

	try:
		p1 = subprocess.Popen(['ps', 'ax'], stdout=subprocess.PIPE)
		p2 = subprocess.Popen(['grep', 'measure'], 
			stdin=p1.stdout, stdout=subprocess.PIPE)
		p1.stdout.close()
		p3 = subprocess.Popen(['wc', '-l'],        
			stdin=p2.stdout,stdout=subprocess.PIPE)
		p2.stdout.close()
		output = p3.communicate()[0]
	except:
		output = 'Nothing'

	output = output.decode('UTF-8').strip() + '#'
	return HttpResponse(output)

def debugStmts(request):
	output = 'Debug Statements:'
	try:
		fo = open("/tmp/debug", "r")
		lines = fo.readlines()
		output = '<br />'.join(lines)
		fo.close()
	except:
		output = ''
	return HttpResponse(output)
