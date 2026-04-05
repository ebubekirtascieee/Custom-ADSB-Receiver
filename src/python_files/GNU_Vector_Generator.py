import pyModeS as pms

# The 88-bit payloads (without the 24-bit CRC)
payloads = [
    "8D4840D6202CC371C32CE0", # TC 4 (Ident)
    "8D4840D658968D56A49520", # TC 11 (Position over Özyeğin)
    "8D4840D69911D408E01080"  # TC 19 (Velocity/Heading)
]

valid_hexes = []
print("Recalculating 24-bit Parity Checksums...")

for p in payloads:
    # encode=True calculates the mathematically valid CRC for the payload
    crc = pms.crc(p + "000000", encode=True)
    valid_hex = f"{p}{crc:06X}"
    valid_hexes.append(valid_hex)
    print(f"Valid Hex: {valid_hex} (CRC: {crc:06X})")

print("\n--- PASTE THIS ENTIRE STRING INTO GNU RADIO VECTOR SOURCE ---")
vector_string = " + [0]*50000 + ".join([f"[1,0,1,0,0,0,0,1,0,1,0,0,0,0,0,0] + sum([[1,0] if c=='1' else [0,1] for c in f\"{{int('{h}', 16):0112b}}\"], [])" for h in valid_hexes]) + " + [0]*50000"
print(vector_string)