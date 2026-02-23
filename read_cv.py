
import os

path = r'c:\Users\Kenura\Documents\TCD1304_CCD_480MHz_FINAL\ml_model\training_plots\cv_output_20fold.txt'
if os.path.exists(path):
    with open(path, 'rb') as f:
        content = f.read().decode('utf-16le')
    print(content)
else:
    print("File not found.")
