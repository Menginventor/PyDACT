b = b'[\xb0C]'
print(b)
b = b.replace(b'\xb0',b'\xc2\xb0')
print(b.decode('utf-8'))