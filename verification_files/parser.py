import os
import re
import argparse

def process_systemverilog_file(input_file, output_file):
    """
    Process a SystemVerilog file by:
    1. Removing all $fwrite blocks
    2. Adding sequential numbering to all assert statements
    
    Args:
        input_file (str): Path to the input SystemVerilog file
        output_file (str): Path to the output processed file
    """
    # Read the input file
    with open(input_file, 'r', encoding='utf-8') as f:
        content = f.read()
    
    # Step 1: Remove all $fwrite blocks
    fwrite_pattern = r'\$fwrite\s*\(.*?\)\s*;'
    content = re.sub(fwrite_pattern, '', content, flags=re.DOTALL)
    
    # Step 2: Add numbering to assert statements
    assert_pattern = r'(assert\s*\(.*?\)\s*;)'
    
    # Find all assert statements
    assert_statements = re.findall(assert_pattern, content, flags=re.DOTALL)
    
    # Replace each assert with numbered version
    for i, assert_stmt in enumerate(assert_statements, start=1):
        numbered_assert = f'assert_verify_no{i}: {assert_stmt}'
        content = content.replace(assert_stmt, numbered_assert, 1)  # Replace one at a time
    
    # Write the processed content to the output file
    with open(output_file, 'w', encoding='utf-8') as f:
        f.write(content)
    # Return the number of assert statements found
    return len(assert_statements)

def generate_sby_files(num_asserts, results_dir, template_file, output_file):
    """
    Generate SBY files for each assert statement based on the template
    
    Args:
        num_asserts (int): Number of assert statements found
        results_dir (str): Directory to save SBY files
        template_file (str): Path to SBY template file
        output_file (str): Path to the output processed file
    """
    # Read the template file
    with open(template_file, 'r', encoding='utf-8') as f:
        template = f.read()
    
    # Create results directory if it doesn't exist
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    
    # Extract just the filename from the output_file path
    output_filename = os.path.basename(output_file)
    
    # Generate one SBY file for each assert statement
    for i in range(1, num_asserts + 1):
        assert_label = f'assert_verify_no{i}'
        
        # Replace placeholders in the template
        sby_content = template
        sby_content = sby_content.replace('{assert_label}', assert_label)
        sby_content = sby_content.replace('NutCore_assert_renamed.sv', output_filename)
        
        # Write the SBY file
        sby_filename = os.path.join(results_dir, f'SimTop_{assert_label}.sby')
        with open(sby_filename, 'w', encoding='utf-8') as f:
            f.write(sby_content)

def main():
    # Set up command line arguments
    parser = argparse.ArgumentParser(description='Process SystemVerilog file and generate SBY files')
    parser.add_argument('input_file', help='Input SystemVerilog file path')
    parser.add_argument('output_file', help='Output SystemVerilog file path')
    parser.add_argument('results_dir', help='Directory for SBY results')
    parser.add_argument('--template', default='SimTop_template.sby', 
                        help='SBY template file (default: SimTop_template.sby)')
    
    args = parser.parse_args()
    
    # Process the SystemVerilog file and get the number of asserts
    num_asserts = process_systemverilog_file(args.input_file, args.output_file)
    
    # Generate the SBY files using the specified template and output file
    generate_sby_files(num_asserts, args.results_dir, args.template, args.output_file)
    
    print(f"Processed {num_asserts} assert statements and generated corresponding SBY files.")
    print(f"Input file: {args.input_file}")
    print(f"Output file: {args.output_file}")
    print(f"SBY files saved to: {args.results_dir}")
    print(f"Used template: {args.template}")

if __name__ == "__main__":
    main()