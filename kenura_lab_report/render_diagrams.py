#!/usr/bin/env python3
"""Render PlantUML blocks from physics_report.qmd via PlantUML server."""
import zlib, base64, urllib.request, re, os, string

report_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'physics_report.qmd')
img_dir = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'images', 'diagrams')
os.makedirs(img_dir, exist_ok=True)

with open(report_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Find all plantuml blocks
pattern = r'```\{plantuml\}\s*\r?\n(.*?)```'
matches = list(re.finditer(pattern, content, re.DOTALL))
print(f'Found {len(matches)} PlantUML blocks')

names = ['system_connectivity', 'firmware_design', 'auto_measurement', 'auto_exposure', 'diagram_5']

def plantuml_encode(text):
    """Encode text using PlantUML's custom encoding."""
    compressed = zlib.compress(text.encode('utf-8'))[2:-4]  # strip zlib header/checksum
    # PlantUML uses custom base64 alphabet
    res = ""
    for i in range(0, len(compressed), 3):
        if i + 2 < len(compressed):
            b1, b2, b3 = compressed[i], compressed[i+1], compressed[i+2]
        elif i + 1 < len(compressed):
            b1, b2, b3 = compressed[i], compressed[i+1], 0
        else:
            b1, b2, b3 = compressed[i], 0, 0
        
        c1 = b1 >> 2
        c2 = ((b1 & 0x3) << 4) | (b2 >> 4)
        c3 = ((b2 & 0xF) << 2) | (b3 >> 6)
        c4 = b3 & 0x3F
        
        res += encode6bit(c1) + encode6bit(c2) + encode6bit(c3) + encode6bit(c4)
    return res

def encode6bit(b):
    if b < 10: return chr(48 + b)  # 0-9
    b -= 10
    if b < 26: return chr(65 + b)  # A-Z
    b -= 26
    if b < 26: return chr(97 + b)  # a-z
    b -= 26
    if b == 0: return '-'
    if b == 1: return '_'
    return '?'

for i, m in enumerate(matches):
    puml = m.group(1).strip()
    name = names[i] if i < len(names) else f'diagram_{i}'
    
    encoded = plantuml_encode(puml)
    url = f'https://www.plantuml.com/plantuml/png/{encoded}'
    out_path = os.path.join(img_dir, f'{name}.png')
    
    try:
        req = urllib.request.Request(url, headers={'User-Agent': 'Mozilla/5.0'})
        with urllib.request.urlopen(req) as resp:
            data = resp.read()
            with open(out_path, 'wb') as fout:
                fout.write(data)
        size = os.path.getsize(out_path)
        print(f'  [{i}] {name}.png - {size} bytes OK')
    except Exception as e:
        print(f'  [{i}] FAILED: {e}')

print('Done!')
