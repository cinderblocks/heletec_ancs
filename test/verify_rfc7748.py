#!/usr/bin/env python3
"""
Standalone RFC 7748 X25519 verification using ONLY Python standard library.
Implements X25519 directly from the RFC 7748 pseudocode using Python big ints.

Run:  python3 test/verify_rfc7748.py

This will print the correct test vector bytes for use in test_mesh_crypto.cxx.
"""

p = 2**255 - 19
a24 = 121665

def clamp(k_bytes):
    k = bytearray(k_bytes)
    k[0] &= 248
    k[31] &= 127
    k[31] |= 64
    return bytes(k)

def decode_u_coordinate(u_bytes):
    # RFC 7748 §5: decode little-endian, mask bit 255
    u = int.from_bytes(u_bytes, 'little')
    u &= (2**255 - 1)
    return u

def encode_u_coordinate(u):
    return (u % p).to_bytes(32, 'little')

def x25519(k_bytes, u_bytes):
    """RFC 7748 §5 X25519 function — direct from the pseudocode."""
    k = clamp(k_bytes)
    scalar = int.from_bytes(k, 'little')
    u = decode_u_coordinate(u_bytes)

    x_1 = u
    x_2 = 1
    z_2 = 0
    x_3 = u
    z_3 = 1
    swap = 0

    for t in range(254, -1, -1):
        k_t = (scalar >> t) & 1
        swap ^= k_t
        # Conditional swap
        if swap:
            x_2, x_3 = x_3, x_2
            z_2, z_3 = z_3, z_2
        swap = k_t

        A  = (x_2 + z_2) % p
        AA = (A * A) % p
        B  = (x_2 - z_2) % p
        BB = (B * B) % p
        E  = (AA - BB) % p
        C  = (x_3 + z_3) % p
        D  = (x_3 - z_3) % p
        DA = (D * A) % p
        CB = (C * B) % p
        x_3 = pow(DA + CB, 2, p)
        z_3 = (x_1 * pow(DA - CB, 2, p)) % p
        x_2 = (AA * BB) % p
        z_2 = (E * (AA + a24 * E)) % p

    # Final conditional swap
    if swap:
        x_2, x_3 = x_3, x_2
        z_2, z_3 = z_3, z_2

    result = (x_2 * pow(z_2, p - 2, p)) % p
    return encode_u_coordinate(result)

def hex_to_bytes(h):
    return bytes.fromhex(h.replace(' ', ''))

def bytes_to_c_array(b, name="data"):
    """Format bytes as a C array initializer."""
    lines = []
    for i in range(0, 32, 8):
        chunk = b[i:i+8]
        hex_vals = ','.join(f'0x{byte:02x}' for byte in chunk)
        lines.append(f'    {hex_vals}{"," if i+8 < 32 else ""}')
    return '\n'.join(lines)

# ── §5.2 Vector 1 — sanity check ────────────────────────────────────────
print("=" * 72)
print("RFC 7748 §5.2 Test Vector 1 (sanity check)")
print("=" * 72)

scalar_52 = hex_to_bytes("a546e36bf0527c9d3b16154b82465edd62144c0ac1fc5a18506a2244ba449ac4")
u_52      = hex_to_bytes("e6db6867583030db3594c1a424b15f7c726624ec26b3353b10a903a6d0ab1c4c")
expect_52 = hex_to_bytes("c3da55379de9c6908e94ea4df28d084f32eccf03491c71f754b4075577a28552")

result_52 = x25519(scalar_52, u_52)
ok = result_52 == expect_52
print(f"  Result:   {result_52.hex()}")
print(f"  Expected: {expect_52.hex()}")
print(f"  {'PASS ✓' if ok else 'FAIL ✗  — Python implementation is broken!'}")
if not ok:
    raise SystemExit(1)

# ── §5.2 Iterated (1 iteration) ─────────────────────────────────────────
print()
print("RFC 7748 §5.2 Iterated (1 iteration): X25519(9, 9)")
nine = b'\x09' + b'\x00' * 31
iter1_expected = hex_to_bytes("422c8e7a6227d7bca1350b3e2bb7279f7897b87bb6854b783c60e80311ae3079")
iter1 = x25519(nine, nine)
ok = iter1 == iter1_expected
print(f"  Result:   {iter1.hex()}")
print(f"  Expected: {iter1_expected.hex()}")
print(f"  {'PASS ✓' if ok else 'FAIL ✗'}")

# ── §6.1 Alice ──────────────────────────────────────────────────────────
print()
print("=" * 72)
print("RFC 7748 §6.1 — Alice")
print("=" * 72)

# The private key as it appears in our test file:
alice_priv_current = hex_to_bytes("77076d0a7318a57d3c16c17251b26645df4c543b268ed7a3c4f2bdd164e09f15")
alice_pub_expected  = hex_to_bytes("8520f0098930a754748b7ddcb43ef75a0dbf3a0d26381af4eba4a98eaa9b4e6a")

alice_pub_computed = x25519(alice_priv_current, nine)
ok = alice_pub_computed == alice_pub_expected
print(f"  Private:  {alice_priv_current.hex()}")
print(f"  Pub got:  {alice_pub_computed.hex()}")
print(f"  Pub want: {alice_pub_expected.hex()}")
print(f"  {'PASS ✓' if ok else 'FAIL ✗  — private key bytes are WRONG'}")

# ── §6.1 Bob ────────────────────────────────────────────────────────────
print()
print("=" * 72)
print("RFC 7748 §6.1 — Bob")
print("=" * 72)

bob_priv_current = hex_to_bytes("5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2689c675585eb8")
bob_pub_expected  = hex_to_bytes("de9edb7d7b7dc1b4d35b61c2ece435373f8343c85b78674dadfc7e146f882b4f")

bob_pub_computed = x25519(bob_priv_current, nine)
ok = bob_pub_computed == bob_pub_expected
print(f"  Private:  {bob_priv_current.hex()}")
print(f"  Pub got:  {bob_pub_computed.hex()}")
print(f"  Pub want: {bob_pub_expected.hex()}")
print(f"  {'PASS ✓' if ok else 'FAIL ✗  — private key bytes are WRONG'}")

# ── §6.1 Shared secret ──────────────────────────────────────────────────
print()
shared_expected = hex_to_bytes("4a5d9d5ba4ce2de1728e3bf480350f25e07e21c947d19e3376f09b3c1e161742")
shared_ab = x25519(alice_priv_current, bob_pub_expected)
shared_ba = x25519(bob_priv_current, alice_pub_expected)
print(f"  X25519(alice_priv, bob_pub):  {shared_ab.hex()}")
print(f"  X25519(bob_priv, alice_pub):  {shared_ba.hex()}")
print(f"  Expected shared:              {shared_expected.hex()}")

# ── Try to find correct keys by brute-checking known variants ────────────
print()
print("=" * 72)
print("Trying known RFC 7748 private key variants...")
print("=" * 72)

# These are various transcriptions of Alice's private key that have appeared
alice_variants = [
    "77076d0a7318a57d3c16c17251b26645df4c543b268ed7a3c4f2bdd164e09f15",
    "77076d0a7318a57d3c16c17251b26645df949d789577965b83f63f2e9fc282e5",
    "77076d0a7318a57d3c16c17251b26645df4949d789577965b83f63f2e9fc2825",
]

bob_variants = [
    "5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2689c675585eb8",
    "5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c268a9c6751daae",
    "5dab087e624a8a4b79e17f8b83800ee66f3bb1292618b6fd1c2689c6751a5768",
]

for v in alice_variants:
    priv = hex_to_bytes(v)
    pub = x25519(priv, nine)
    match = "✓ MATCH" if pub == alice_pub_expected else ""
    print(f"  Alice {v} → {pub.hex()[:16]}... {match}")

for v in bob_variants:
    priv = hex_to_bytes(v)
    pub = x25519(priv, nine)
    match = "✓ MATCH" if pub == bob_pub_expected else ""
    print(f"  Bob   {v} → {pub.hex()[:16]}... {match}")

print()
print("If none match, the correct private key bytes need to be obtained")
print("from the actual RFC 7748 document or a trusted implementation.")
print()
print("Try also:  pip3 install cryptography && python3 -c \"")
print("from cryptography.hazmat.primitives.asymmetric.x25519 import X25519PrivateKey")
print("import sys")
print("for line in sys.stdin:")
print("    k=X25519PrivateKey.from_private_bytes(bytes.fromhex(line.strip()))")
print("    print(k.public_key().public_bytes_raw().hex())\"")
