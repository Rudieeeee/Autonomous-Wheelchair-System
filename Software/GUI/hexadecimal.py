def decimal_to_hex(decimal_value):
    """
    Convert a decimal value to a 2-digit hexadecimal value.
    For negative numbers, takes the last two digits of the signed hex representation.
    For positive numbers, takes the last two digits of the hex representation.
    
    Args:
        decimal_value (int): The decimal value to convert
        
    Returns:
        str: Two-digit hexadecimal value
    """
    # Convert to signed 16-bit integer
    if decimal_value < 0:
        # For negative numbers, convert to signed 16-bit hex
        hex_value = format(decimal_value & 0xFFFF, '04X')
    else:
        # For positive numbers, convert to hex
        hex_value = format(decimal_value, '04X')
    
    # Return only the last two digits
    return hex_value[-2:]

# Example usage:
if __name__ == "__main__":
    # Test cases
    test_values = [-56, 100, -50, 50, 0]
    for value in test_values:
        hex_result = decimal_to_hex(value)
        print(f"Decimal: {value} -> Hex: {hex_result}")