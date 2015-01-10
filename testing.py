import cookielib
import urllib
import urllib2
from bs4 import BeautifulSoup
import urllib
import os
import os.path
# Store the cookies and create an opener that will hold them
cj = cookielib.CookieJar()
opener = urllib2.build_opener(urllib2.HTTPCookieProcessor(cj))

# Add our headers
opener.addheaders = [('User-agent', 'Python-rocks')]

# Install our opener (note that this changes the global opener to the one
# we just made, but you can also just call opener.open() if you want)
urllib2.install_opener(opener)

# The action/ target from the form
authentication_url = "http://moodle.iitb.ac.in/login/index.php"

# Input parameters we are going to send
payload = {
  'username':'120050061',
  'password': 'cute$dheeru'
  }

# Use urllib to encode the payload
data = urllib.urlencode(payload)

# Build our Request object (supplying 'data' makes it a POST)
req = urllib2.Request(authentication_url, data)

# Make the request and read the response
resp = urllib2.urlopen(req)
contents = resp.read()
contents
alpha=contents.split("<h2>My courses</h2>")[1]
alpha=alpha.split('<aside id="block-region-side-pre" ')[0];
import re
r = re.compile('<h3 class="coursename">(.*)<div class="moreinfo">')
m = r.search(alpha)
if m:
	lyrics = m.group(1)
#print m.group(1)
delta=m.group(1)
soup=BeautifulSoup(delta)
soup.findAll('a')
listurl=[]
for x in soup.findAll('a'):
	listurl.append(x.get('href'))
print listurl

for y in listurl:
	resp1=urllib2.urlopen(y)
	scrap=resp1.read()
	soup=BeautifulSoup(scrap)
	#print soup.findAll('a')
	pdflist=[]
	for x in soup.findAll('a'):
		if (x.get('href') is None):
			continue
		if('resource' in (x.get('href'))):
			pdflist.append(x.get('href'))
	print pdflist
	for t in pdflist:
		cj1 = cookielib.CookieJar()
		opener1 = urllib2.build_opener(urllib2.HTTPCookieProcessor(cj1))

		# Add our headers
		opener1.addheaders = [('User-agent', 'Python-rocks')]

		# Install our opener (note that this changes the global opener to the one
		# we just made, but you can also just call opener.open() if you want)
		urllib2.install_opener(opener1)

		# The action/ target from the form
		authentication_url1 = t

		# Input parameters we are going to send
		payload1 = {
		  'username':'120050061',
		  'password': 'cute$dheeru'
		  }

		# Use urllib to encode the payload
		data1 = urllib.urlencode(payload1)

		# Build our Request object (supplying 'data' makes it a POST)
		req1 = urllib2.Request(authentication_url, data)

		# Make the request and read the response
		resp1 = urllib2.urlopen(req1)
		print resp1.read()
		webFile = urllib2.urlopen(t)

		if (os.path.isfile(t.split('/')[-1])):
			continue
		pdfFile = open(t.split('/')[-1], 'w')
		pdfFile.write(webFile.read())
		webFile.close()
		pdfFile.close()



