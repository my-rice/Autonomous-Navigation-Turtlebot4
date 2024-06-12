import getpass
import subprocess

# Ask for the password
password = getpass.getpass("Enter your password: ")

# Define the command
command = 'sudo -S ntpdate ntp.ubuntu.com'

# Run the command
proc = subprocess.Popen(['sudo', '-S'] + command.split(), stdin=subprocess.PIPE, stderr=subprocess.PIPE)
sudo_prompt = proc.communicate(password.encode())[1]


# Define the command
command = 'cd ./../scripts; ./configure_discovery.sh'

# Define the inputs
inputs = '192.168.118.193\n\n\n'

# Run the command
proc = subprocess.Popen(command, stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
stdout, stderr = proc.communicate(inputs.encode())

print(stdout.decode())

# Define the command
command = 'ros2 daemon stop; ros2 daemon start; ros2 topic list'

# Run the command
proc = subprocess.Popen(command, stdout=subprocess.PIPE, stderr=subprocess.PIPE, shell=True)
stdout, stderr = proc.communicate()

print(stdout.decode())