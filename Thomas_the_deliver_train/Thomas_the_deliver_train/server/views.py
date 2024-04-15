from django.shortcuts import render,HttpResponse,HttpResponseRedirect
import os

import paramiko
import random

host = "172.20.10.4"
username = "ubuntu"
password = "raspberrypi"

client = paramiko.client.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(host, username=username, password=password)

list=[0,1,2,3,4,5,6,7,8,9]
num=str(random.choice(list)) + str(random.choice(list)) + str(random.choice(list)) + str(random.choice(list))

# Create your views here.
def index(request):
    return render(request,"basic.html")

def save_gps(request):
    if request.method != "POST":
        return index(request)

    gps_info = request.POST.get("gps_info")

    # File created on local desktop
    f=open("C:/Users/Andrew/Downloads/Thomas_the_deliver_train/Thomas_the_deliver_train/server/coordinates.txt","w")
    f.write(gps_info+" "+num)
    f.close()

    ftp = client.open_sftp()
    ftp.put("C:/Users/Andrew/Downloads/Thomas_the_deliver_train/Thomas_the_deliver_train/server/coordinates.txt","/home/ubuntu/Desktop/coordinates.txt")
    ftp.close()

    path = os.path.join('upload',"gps.txt")
    print(path)
    with open("gps.txt",'w') as f:
        f.write(gps_info)
        f.close()

    return index(request)