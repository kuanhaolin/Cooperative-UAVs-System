import pypandoc

def convert_to_pdf(input_file, output_file):
    try:
        output = pypandoc.convert_file(input_file, 'pdf', outputfile=output_file)
        print("Conversion successful:", input_file, "to", output_file)
    except Exception as e:
        print("Error during conversion:", e)

if __name__ == "__main__":
    convert_to_pdf("proposal.docx", "proposal.pdf")