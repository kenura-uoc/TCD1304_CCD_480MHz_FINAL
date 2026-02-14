
import sys
try:
    import pypdf
except ImportError:
    try:
        import PyPDF2 as pypdf
    except ImportError:
        print("Please install pypdf: pip install pypdf")
        sys.exit(1)

def extract_text(pdf_path, output_path):
    reader = pypdf.PdfReader(pdf_path)
    text = ""
    for page in reader.pages:
        text += page.extract_text() + "\n"
    
    with open(output_path, "w", encoding="utf-8") as f:
        f.write(text)
    print(f"Saved text to {output_path}")

if __name__ == "__main__":
    pdf_path = "10-Chem.Eng.Com..pdf"
    output_path = "paper.txt"
    extract_text(pdf_path, output_path)
