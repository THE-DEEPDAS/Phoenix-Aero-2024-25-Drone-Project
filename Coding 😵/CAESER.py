def caesar_cipher_encrypt(text):
    result = ""
    for char in text:
        if char.isalpha():
            if char.isupper():
                result += chr(155 - ord(char))
                # 65 to 90 aave ne A to Z etle add karine minus kairu che.
            else:
                result += chr(219 - ord(char))
        else:
            # biju kai aave to direct muki devanu
            result += char

    return result

message = "HELLO WORLD this IS a pROSoUnD method."
encrypted_message = caesar_cipher_encrypt(message)
print("Original message: ", message)
print("Encrypted message: ", encrypted_message)
message2 = "abcdefghijklmnopqrstuvwxyz."
encrypted_message2 = caesar_cipher_encrypt(message2)
print("Original message: ", message2)
print("Encrypted message: ", encrypted_message2)