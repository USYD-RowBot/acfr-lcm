import subprocess

p = subprocess.Popen(['ls', '-l', '-h'], stdin=subprocess.PIPE, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
output, err = p.communicate()
rc = p.returncode

print rc
print output