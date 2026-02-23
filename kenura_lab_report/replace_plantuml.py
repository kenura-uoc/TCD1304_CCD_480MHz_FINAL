#!/usr/bin/env python3
"""Replace PlantUML code blocks in physics_report.qmd with static image references."""
import re, os

report_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'physics_report.qmd')

with open(report_path, 'r', encoding='utf-8') as f:
    content = f.read()

# Find all plantuml blocks
pattern = r'```\{plantuml\}\s*\r?\n.*?@enduml\s*\r?\n```'
matches = list(re.finditer(pattern, content, re.DOTALL))
print(f'Found {len(matches)} PlantUML blocks to replace')

# Map each block to its image replacement
replacements = [
    ('system_connectivity', 'System connectivity diagram showing physical and logical connections between STM32H743VIT6, TCD1304 CCD, lasers, servo, LCD, SD card, and USB', 'fig-sys-connectivity'),
    ('firmware_design', 'Software architecture class diagram showing the modular firmware design with Core Application, Signal Processing, and Storage packages', 'fig-firmware-arch'),
    ('auto_measurement', 'State machine diagram for the automated dual-laser measurement sequence', 'fig-auto-measure-sm'),
    ('auto_exposure', 'State machine diagram for the auto-exposure P-control optimization loop', 'fig-auto-exposure-sm'),
    ('diagram_5', 'Additional system diagram', 'fig-diagram-5'),
]

# Replace from last to first to preserve positions
for i in range(len(matches) - 1, -1, -1):
    m = matches[i]
    name, caption, label = replacements[i]
    img_ref = f'![{caption}](images/diagrams/{name}.png){{#{label}}}'
    content = content[:m.start()] + img_ref + content[m.end():]
    print(f'  [{i}] Replaced block at pos {m.start()}-{m.end()} with {name}.png')

with open(report_path, 'w', encoding='utf-8') as f:
    f.write(content)

print('Done! All PlantUML blocks replaced with static images.')
