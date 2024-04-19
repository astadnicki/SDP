import paramiko
import random

# Update the next three lines with your
# server's information

host = "172.20.10.4"
username = "ubuntu"
password = "raspberrypi"

client = paramiko.client.SSHClient()
client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
client.connect(host, username=username, password=password)

# User input
coords=input('Enter coordinates (xx yy zz): ')

list=[0,1,2,3,4,5,6,7,8,9]
num=str(random.choice(list)) + str(random.choice(list)) + str(random.choice(list)) + str(random.choice(list))

# File created on local desktop
f=open("coordinates.txt","w")
f.write(coords+" "+num)
f.close()

ftp = client.open_sftp()
ftp.put("C:/Users/Andrew/Downloads/SDP_SSH/coordinates.txt","/home/ubuntu/Desktop/coordinates.txt")
ftp.close()

#_stdin, _stdout,_stderr = client.exec_command("df")
#print(_stdout.read().decode())
client.close()