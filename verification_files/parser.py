import os
import re

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

def generate_sby_files(num_asserts):
    """
    Generate SBY files for each assert statement based on the template
    
    Args:
        num_asserts (int): Number of assert statements found
    """
    # Read the template file
    with open('SimTop_template.sby', 'r', encoding='utf-8') as f:
        template = f.read()
    
    # Create results directory if it doesn't exist
    results_dir = 'results'
    if not os.path.exists(results_dir):
        os.makedirs(results_dir)
    
    # Generate one SBY file for each assert statement
    for i in range(1, num_asserts + 1):
        assert_label = f'assert_verify_no{i}'
        
        # Replace the placeholder in the template
        sby_content = template.replace('{assert_label}', assert_label)
        
        # Write the SBY file
        sby_filename = os.path.join(results_dir, f'SimTop_{assert_label}.sby')
        with open(sby_filename, 'w', encoding='utf-8') as f:
            f.write(sby_content)

def main():
    # Process the SystemVerilog file and get the number of asserts
    num_asserts = process_systemverilog_file('./NutCore.sv', './NutCore_assert_renamed.sv')
    
    # Generate the SBY files
    generate_sby_files(num_asserts)
    
    print(f"Processed {num_asserts} assert statements and generated corresponding SBY files.")

if __name__ == "__main__":
    main()
